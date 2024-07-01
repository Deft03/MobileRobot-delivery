import socket
import threading
import math
import path4server

HOST = "192.168.1.166"
PORT = 2113
ADR = (HOST,PORT)

HEADER = 64
DISCONNECT_MESSAGE = "!DISCONNECT"

class Agent:
    def __init__(self,id,x,y,status):
        self.id = id
        self.curr_x = x
        self.curr_y = y
        self.status = status  # format status
    def updatePositionStatus(self,x,y,status):
        self.curr_x = x
        self.curr_y = y
        self.status = status
    def showInfo(self):
        print(f"ID: {self.id}|| Status: {self.status}|| X: {self.curr_x}|| Y: {self.curr_y}")

agent_list = [] ## list all agent
#goal_list = [(2,1),(3,4),(3,2),(6,7)] ## for all agent . isn't it? 
goal_list = [(2,1),(3,4),(3,2),(6,7)]
home_list = [(2,8),(3,5)] ## for all agent . isn't it? 
station_list = [(2,1),(3,4),(3,2),(6,7)] ## for all agent . isn't it? 

# goal_list = []
job_dict = {}
to_make_path = []
path_dict = {}

def calculateDistance(goal_pos,agent_pos):
    gx,gy = goal_pos
    ax,ay = agent_pos
    distance = math.sqrt((gx-ax)**2+(gy-ay)**2)
    return distance

def assign_process():
    free_agent_list = []
    home_agent_list = []
    goal_agent_list = []
    global job_dict
    job_dict = {}
    if(len(goal_list) | len(home_list) ):
        for agent in agent_list:
            if(agent.status==0):
                free_agent_list.append(agent) # home -> station
            if(agent.status==2):
                goal_agent_list.append(agent) # station -> goal
            if(agent.status==3):
                home_agent_list.append(agent) # goal -> home

        print(free_agent_list)                
    if(len(free_agent_list)): 
        for goal in station_list:
            min_dist = 99999
            min_agent = None
            for agent in free_agent_list:
                temp = calculateDistance(goal,(agent.curr_x,agent.curr_y))
                if(temp <= min_dist):
                    min_dist = temp
                    min_agent = agent
            if(min_agent!=None):
                job_dict[goal] = min_agent
                free_agent_list.remove(min_agent)
                # goal_list.remove(goal)
                # print(goal_list)
            else:
                print(f"No availble agent for {goal} goal")
        for goal in job_dict.keys():
            station_list.remove(goal)
    if(len(goal_agent_list)): 
        for goal in goal_list:
            min_dist = 99999
            min_agent = None
            for agent in goal_agent_list:
                temp = calculateDistance(goal,(agent.curr_x,agent.curr_y))
                if(temp <= min_dist):
                    min_dist = temp
                    min_agent = agent
            if(min_agent!=None):
                job_dict[goal] = min_agent
                goal_agent_list.remove(min_agent)
                # goal_list.remove(goal)
                # print(goal_list)
            else:
                print(f"No availble agent for {goal} goal")
        for goal in job_dict.keys():
            goal_list.remove(goal)
    if(len(home_agent_list)):  
        for goal in home_list:
            min_dist = 99999
            min_agent = None
            for agent in home_agent_list:
                temp = calculateDistance(goal,(agent.curr_x,agent.curr_y))
                if(temp <= min_dist):
                    min_dist = temp
                    min_agent = agent
            if(min_agent!=None):
                job_dict[goal] = min_agent
                home_agent_list.remove(min_agent)
                # goal_list.remove(goal)
                # print(goal_list)
            else:
                print(f"No availble agent for {goal} goal")
        for goal in job_dict.keys():
            home_list.remove(goal)


def assignJob():
    free_agent_list = []
    global job_dict
    job_dict = {}
    if(len(goal_list)):
        for agent in agent_list:
            if(agent.status==0): ## TODO: define or enum status
                free_agent_list.append(agent)

        print(free_agent_list)                
    # if(len(free_agent_list)):     
        for goal in goal_list:
            min_dist = 99999
            min_agent = None
            for agent in free_agent_list:
                temp = calculateDistance(goal,(agent.curr_x,agent.curr_y))
                if(temp <= min_dist): 
                    min_dist = temp
                    min_agent = agent
            if(min_agent!=None):
                job_dict[goal] = min_agent
                free_agent_list.remove(min_agent)
                # goal_list.remove(goal)
                print(goal_list)
            else:
                print(f"No availble agent for {goal} goal")
        for goal in job_dict.keys():
            goal_list.remove(goal)



def manageOrder():
    while True:
        to_make_path = []
        temp = input("Press q to quit: ")
        if(temp == "q"):
            break
        # else:
        #     if(len(temp)):
        #         x,y = temp.split(',')
        #         x = int(x)
        #         y = int(y)
        #         temp_goal = (x,y)
        #         goal_list.append(temp_goal)
         # if(len(goal_list)>0 | len(home_list)>0 | len(station_list)>0 ):

        if(len(goal_list)>0):
            assign_process()
        if(len(job_dict)>0):
            for goal,agent in job_dict.items():
                job = {}
                job['name'] = agent.id
                job['start'] = [agent.curr_x,agent.curr_y]
                job['goal'] = list(goal)
                job['even'] = None
                if(job not in to_make_path):
                    to_make_path.append(job)
            print(to_make_path)
            global path_dict 

            path_dict = path4server.main(to_make_path)
            print(path_dict)         


def handleClient(conn,adr):
    print(f"[NEW CONNECTION] {adr} connected")
    connected = True
    listening = True
    robot_id = None
    while connected:
        if(listening):
            msg_length = conn.recv(HEADER).decode('utf-8')
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length).decode('utf-8')
                # conn.send("Msg received".encode('utf-8'))
                if msg == DISCONNECT_MESSAGE:
                    connected = False
                else:
                    id,status,x,y = msg.split('|') ## wow
                    x = int(x)
                    y = int(y)
                    status = int(status)
                    print(id,status,x,y)
                    new_agent = Agent(id,x,y,status)
                    for agent in agent_list:
                        if(id==agent.id):
                            agent.updatePositionStatus(x,y,status)
                            agent.showInfo()
                            break
                    else:
                        agent_list.append(new_agent)
                        robot_id = id
                    if(status==0):
                        listening = False
                    else:
                        listening = True
                print(f"[{adr}] {msg}")
        else:
            global path_dict
            if(len(path_dict)):
                print(robot_id)
                try:
                    print(path_dict[robot_id])
                    path = path_dict[robot_id]
                    path_dict.pop(robot_id)
                    path_str = ""
                    for item in path:
                        path_str += str(item[0])+item[1]
                        path_str += ','
                    path_str = path_str[:-1]
                    print(path_str)
                    conn.send(path_str.encode('utf-8'))
                    listening = True
                except:
                    print("no")

    conn.close()

def start():
    server.listen(5)
    print(f"[LISTENING] Server is listening on {socket.gethostbyname(socket.gethostname())}")
    order_thread = threading.Thread(target=manageOrder)
    order_thread.start()
    while True:
        conn, adr = server.accept()
        thread = threading.Thread(target=handleClient,args=(conn,adr))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.active_count()-2}")
    

server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.bind(ADR)

print("Server is starting!")
start()

