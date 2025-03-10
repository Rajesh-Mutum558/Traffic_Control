import socket
import struct
import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(8,GPIO.OUT, initial=GPIO.LOW)    # green led pin
GPIO.setup(12,GPIO.OUT, initial=GPIO.LOW)   # red led pin
local_ip = "10.114.240.46"

MCAST_GROUP = '230.1.25.5'
MCAST_PORT = 5008

MCAST_TTL = 2
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GROUP), socket.INADDR_ANY)

multicast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
multicast_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MCAST_TTL)
multicast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, MCAST_TTL)
multicast_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
multicast_socket.bind((MCAST_GROUP, MCAST_PORT))
multicast_socket.settimeout(0.1)
time_sync = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def RequestTimefromNtp(addr='ntp.iisc.ac.in'):
    REF_TIME_1970 = 2208988800  # Reference time
    data = b'\x1b' + 47 * b'\0'
    time_sync.sendto(data, (addr, 123))
    data, address = time_sync.recvfrom(1024)
    if data:
        t = struct.unpack('!12I', data)[10]
        t -= REF_TIME_1970
    return t

def read_socket():
    is_recv = True
    try:
        udp_data, address = multicast_socket.recvfrom(1024)
    except socket.timeout:
        is_recv = False
    if is_recv == True:
        sender_IP = address[0]
        udp_data = udp_data.strip()
        udp_data = str(udp_data).split("|")
        seq_no = int(str(udp_data[0][2:]))
        msg_type = int(str(udp_data[1]))
        msg_data = udp_data[2][:len(udp_data[2])-1]
        
        #if all_connect == False:
        print("New Packet has arrived in the Rasberry-Pi")
        print("    \tSender IP : {}".format(sender_IP),end='\n')
        print("    \tSeq no: {}".format(seq_no),end='\n')
        print("    \tMessage type: {}".format(msg_type),end='\n')
        print("    \tMessage data: {}".format(msg_data),end='\n')
        print("=======================================================================",end='\n')
        
        return (sender_IP,seq_no,msg_type,msg_data)
    else:
        return("0.0.0.0",-1,-1,"")


def msg_extraction(sender_IP, seq_no, msg_type, msg_data):
    global user_count
    isfound = False                                     # assume sender not known
    next_msg_type = -1                                  # default next_msg_type
    for i in range(0,user_count):                       # search in present user_IP list 
        if user_IP[i] == sender_IP and seq_no != -1:
            isfound = True
            break
    if isfound == False and seq_no != -1:                                # if not found then
        user_IP.append(sender_IP)                       # append sender_IP in user_IP list
        user_ntp_data.append(0)                         # append default (0) value in user_ntp_data list
        user_count += 1
    
    match msg_type:
        
        case 0:                                         # connection establishment msg
            user_ntp_data.append(int(msg_data))         # update sender's t_ntp in user_ntp_data
            next_msg_type = 1                           # next send it an ack
        
        case 1:                                                                         # connection ack msg
            ack_status[user_IP.index(sender_IP)][user_IP.index(msg_data)] = True        # update in table row = sender_IP, col = recv_IP
        
        case 2:                                         # vehicle traffic data msg
            msg_data = msg_data.split(" ")
            left_vehicles = int(msg_data[0])
            centre_vehicles = int(msg_data[1])
            right_vehicles = int(msg_data[2])
            
            if(vehicle_recv_status[user_IP.index(sender_IP)] == False) and seq_no == loop_counter:              # not received data (new data) and of same seq_no as loop_counter
                vehicle_direction_count[user_IP.index(sender_IP)*3] = left_vehicles
                vehicle_direction_count[user_IP.index(sender_IP)*3 + 1] = centre_vehicles
                vehicle_direction_count[user_IP.index(sender_IP)*3 + 2] = right_vehicles
                vehicle_count[user_IP.index(sender_IP)] += (left_vehicles + centre_vehicles + right_vehicles)
                table[0][user_IP.index(sender_IP)] = vehicle_count[user_IP.index(sender_IP)]                    # table row = 0 total vehicles update
                table[1][user_IP.index(sender_IP)] = 0                                                          # initially assume it will be RED
                vehicle_recv_status[user_IP.index(sender_IP)] = True                                            # update vehicle_recv_status
        
        case 3:                                                             # vehicle ack msg contains IP of user to be GREEN
            if seq_no  == loop_counter:                                     # only if ack for current minute
                green_IP_list[user_IP.index(sender_IP)] = msg_data          # update green_IP_list at sender IP index
                vehicle_ack_status[user_IP.index(sender_IP)] = True         # update vehicle_ack_status

    return next_msg_type


def send_msg(seq_no, msg_type, IP=""):
    data =""
    match msg_type:
        case 0:
            data = str(t_ntp)
        case 1:
            data = IP
        case 2:
            data = str(traffic_data[seq_no-1])     # (seq_no -1) to handle 0 indexing
        case 3:
            data = IP
    
    msg_data = str(seq_no)+"|"+str(msg_type)+"|"+data
    multicast_socket.sendto(bytes(msg_data,encoding='utf8'),(MCAST_GROUP,MCAST_PORT))


def traffic_algo(seq_no):
    index = vehicle_count.index(max(vehicle_count))             # max vehicle count index
    
    if table[3][index] == 3:                                    # if max_vehicle count user has 3 consecutive green
        v_sort=vehicle_count                                    # 2nd max becomes green index
        v_sort.sort()
        print(f"sorted array:{v_sort}")
        index = vehicle_count.index(v_sort[-2])
    
    for j in range(0,4):                                        # if there exist a some user who hasnt been green from last 9 minutes
        if seq_no - table[2][j] == 9:
            index = j
            break
    #print("****")
    return index

def conflict_check(seq_no,index):
    isconflict = False                                          # assume no conflict
    print(f"green IP are : {green_IP_list}")
    for i in range(0,3):
        if green_IP_list[i] != green_IP_list[i+1]:              # compare all green_IP_list values, if any user has different green_IP              
            isconflict = True                                   # indicates conflict
            break
    
    if isconflict == True:                                      # if isonflict then led _status = False i.e. blink will happen
        led_status = False                                      # no stable state
    else:
        table[1][index] = 1                                     # else update table
        table[2][index] = seq_no
        for j in range(0,4):
            if j != index:
                table[3][j] = 0
            else:
                table[3][j] += 1

        global cumulative_counts
        cumulative_counts[index][0] += vehicle_direction_count[index * 3]    #left
        cumulative_counts[index][1] += vehicle_direction_count[index * 3 + 1]  # center
        cumulative_counts[index][2] += vehicle_direction_count[index * 3 + 2]  #right
        print('.....................................................................')
        print(f"Left vehicles: {vehicle_direction_count[index*3]}")
        print(f"Center vehicles: {vehicle_direction_count[index*3 + 1]}")
        print(f"Right vehicles: {vehicle_direction_count[index*3 + 2]}")
        print('.....................................................................')
        print(f"Cumulative Left vehicles: {cumulative_counts[index][0]}")
        print(f"Cumulative Center vehicles: {cumulative_counts[index][1]}")
        print(f"Cumulative Right vehicles: {cumulative_counts[index][2]}")
        vehicles_sub = 0   
        m=0                                             # vehicles to be subtracted from left, centre, right = min(value,10)
        for i in cumulative_counts[index]:
            if i < 10:
                vehicles_sub += i
                cumulative_counts[index][m] -= i
            else:
                vehicles_sub += 10
                cumulative_counts[index][m] -= 10
            m+=1
        print('.....................................................................')
        print("After subtracting the allowable number of vehicles from each side, the remaining vehicles are: ")
        print(f"Left vehicles: {cumulative_counts[index][0]}")
        print(f"Center vehicles: {cumulative_counts[index][1]}")
        print(f"Right vehicles: {cumulative_counts[index][2]}")
        print('.....................................................................')
        print(f"Total vehicles to subtract: {vehicles_sub}")
        print('.....................................................................')
        print((table[0][index],vehicles_sub))
        #print(table[0][index])
        table[0][index] -= vehicles_sub
        print(table[0][index])
        vehicle_count[index] -= vehicles_sub
        led_status = True                                               # set led_status = true
        state_info = open("state_table.txt","w")                        # store current state information in state_table.txt file
        state_info.writelines(str(user_ntp_data)+'\n')
        state_info.writelines(str(user_IP)+'\n')
        state_info.writelines(str(table[0])+'\n')
        state_info.writelines(str(table[1])+'\n')
        state_info.writelines(str(table[2])+'\n')
        state_info.writelines(str(table[3])+'\n')
        state_info.writelines(str(user_count)+'\n')
        state_info.close()
        
    for j in range(0,4):
        print(table[j])
    print(f"Conflict is there or not : {isconflict}")
    
    return led_status


def load_prev_state():
    global user_IP,user_ntp_data,table,vehicle_count,user_count
    file_obj = open("state_table.txt","r")
    prev_state = file_obj.readlines()
    
    for i in range(0,4):
        user_ntp_data.append(int(((prev_state[0].split("[")[1]).split("]")[0]).split(",")[i]))
        user_IP.append(((prev_state[1].split("[")[1]).split("]")[0]).split(",")[i])
        user_IP[i] = user_IP[i].split("'")[1]
    
    for i in range(0,4):
        prev_state[i+2] = (prev_state[i+2].split("["))[1].split("]")[0].split(",")
        for j in range(0,4):
            table[i][j] = int(prev_state[i+2][j])
    
    for i in range(0,4):
        vehicle_count[i] = table[0][i]
    
    user_count = int(prev_state[6])
    #print(user_count)
    #print(table)


# variable declarations
t_ntp = RequestTimefromNtp()
print(f"Time from NTP: {time.ctime(t_ntp)}")
t_err = time.time()- t_ntp
print(t_err)
user_IP = []                                    # list of user IPs
user_ntp_data = []                              # corresponding list of their t_ntp
ack_status = [[True,False,False,False],
              [False,True,False,False],
              [False,False,True,False],
              [False,False,False,True]]

cumulative_counts = [
    [0, 0, 0],  # Raspberry Pi 1: [left_count, center_count, right_count]
    [0, 0, 0],  # Raspberry Pi 2: [left_count, center_count, right_count]
    [0, 0, 0],  # Raspberry Pi 3: [left_count, center_count, right_count]
    [0, 0, 0]  ] # Raspberry Pi 4: [left_count, center_count, right_count]
user_count = 0
file_obj = open(r"queue.txt","r")
traffic_data = file_obj.readlines()
isrestart = False                               # flag mentioning whether it is restart or not after shut down

table = [[0,0,0,0],             # table cols are different users while table row = 0 is total vehicles count
         [0,0,0,0],             # row = 1 is current status(GREEN or RED)
         [0,0,0,0],             # row = 2 is last when it was green time
         [0,0,0,0]]             # row = 3 is consecutive green count

vehicle_count = [0,0,0,0]                                       # current total vehicles for different users
vehicle_direction_count = [0,0,0,0,0,0,0,0,0,0,0,0]             # vehicle count left, centre, right for each user
vehicle_recv_status = [False,False,False,False]                 # vehicle data recive status
vehicle_ack_status = [False,False,False,False]                  # vehicle data acknowledgement status
green_IP_list = [0,0,0,0]                                       # list containg calculated GREEN IP by each user
isledset = False                                                # led set flag
isledblink = False                                              # led blink flag


### Connection establishment stage
all_connect = False
t_count = time.time()
print("trying to establish connection with all other hosts ...")

while all_connect == False:
    if time.time() > t_count + 3:
        send_msg(0,0)                                           # send 1 message containing ntp_time every 3 sec def send_msg(seq_no, msg_type, IP="")
        t_count = time.time()                                   # update t_count to current time (for 3 sec calc)
        print("all_connect variable value : ",all_connect)
        print("ack_status : ", ack_status)
    sender_IP,seq_no,msg_type,msg_data = read_socket()          # Read msg (timeout = 0.1sec)
    
    if seq_no > 0:                                              # if recv msg seq_no > 0 => Pi was shutdown
        load_prev_state()
        loop_counter = seq_no                                   # loop_counter will be that minute as indicated by seq_no
        all_connect = True
        isrestart = True
        break

    if seq_no != -1:                                                         # new msg found
        next_msg_type = msg_extraction(sender_IP,seq_no,msg_type,msg_data)
        if next_msg_type == 1:                                               # new t_ntp msg received
            send_msg(seq_no,next_msg_type,sender_IP)                         # then send ack
    
    all_connect = True
    for i in range(0,4):
        for j in range(0,4):
            all_connect = all_connect & ack_status[i][j]
    #all_connect = ack_status[0][1] & ack_status[1][0]                         # update all_connect value
    

print("connection_established")
if isrestart == False:
    loop_counter = 1
t_start = max(user_ntp_data) + t_err + 30*(loop_counter+1)      # 15 second to sync and connect establish establish connection above
t_3_sec = t_start                                               # start of every minute will be used after operation start
print("Traffic control operations start in " + str(t_start - time.time()) + " secs")

while time.time() < t_start:
    for i in user_IP:
        send_msg(0,1,i)
        time.sleep(0.5)
### traffic data can be shared.

while loop_counter < len(traffic_data):                         #len(tranffic_data) gives number of rows in queue.txt
    
    all_recv = vehicle_recv_status[0]&vehicle_recv_status[1]&vehicle_recv_status[2]&vehicle_recv_status[3]
    all_ack = vehicle_ack_status[0]&vehicle_ack_status[1]&vehicle_ack_status[2]&vehicle_ack_status[3]
    #all_recv = vehicle_recv_status[0]&vehicle_recv_status[1]
    #all_ack = vehicle_ack_status[0]&vehicle_ack_status[1]

    if (time.time() - (t_3_sec)) >= 3:
        if all_ack == False:
            send_msg(loop_counter,2)                                    # send traffic data once every 3 sec
        print(f"light is set : {isledset}")
        if all_recv == True and isledset == False:                                            # if all vehicle data received
            green_index = traffic_algo(loop_counter)                    # calculate green index (traffic_algo())
            print(f"green Index Value : {green_index}")
            print(f"IP : {user_IP}")
            print("current green signal has ip", user_IP[green_index])
            send_msg(loop_counter,3,user_IP[green_index])               # vehicle ack , multicast current green ip to all host
        t_3_sec = time.time()                                           # update t_3_sec start point
        if all_recv == True:
            send_msg(loop_counter,3,user_IP[green_index])

    sender_IP,seq_no,msg_type,msg_data = read_socket()                      # read incoming messages includes traffic data as well as ack for traffic data
    next_msg_type = msg_extraction(sender_IP,seq_no,msg_type,msg_data)
    
    if all_ack == True and isledset == False:                           # if aal users have ack and led is yet not set
        led_status = conflict_check(loop_counter,green_index)           # check for conflict
        isledset = True                                                 # isledset = true
        print(f"Light is set or not : {isledset}")
    
    if isledblink == True:                                              # if led blink
        if (time.time() -t_start)%2 <1:                                 # then RED ON for odd sec
            GPIO.output(8, GPIO.LOW)                                    
            GPIO.output(12, GPIO.HIGH)
        else: 
            GPIO.output(12, GPIO.LOW)                                   # and RED OFF for even sec

    if (time.time() - (t_start)) > 30:                          # currently 30 sec = 1min fast forward mode to check, increment loop counter(traffic data row)    
        if all_ack == True and all_recv == True :               # if all_ack and all_recv are true
            isledblink  =  False                                # led blink wont happen
            if local_ip==user_IP[green_index]:                  # check your IP status and set RED or GEEEN accordingly
                GPIO.output(8, GPIO.HIGH)
                GPIO.output(12, GPIO.LOW)
            else: 
                GPIO.output(8, GPIO.LOW)
                GPIO.output(12, GPIO.HIGH)
        else:                                                   # else it is led blink
            isledblink = True

        print("new minute starts "+str(loop_counter))
        #traffic_algo(loop_counter)
        # vehicle_recv_status=[False,False,False,False]
        # vehicle_ack_status=[False,False,False,False]
        green_IP_list = [0,0,0,0]
        if isledset == True:                                    # only if isledset reset all status variables and increment to next min
            loop_counter += 1                                   # otherwise it leads to double addition of vehicle count
            vehicle_recv_status=[False,False,False,False]
            vehicle_ack_status=[False,False,False,False]
        isledset = False
        t_start = time.time()                                   # update t_start timer
    
