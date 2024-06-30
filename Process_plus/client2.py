import socket
import time

HOST = "192.168.19.109"
PORT = 2113
ADR = (HOST,PORT)

HEADER = 64
DISCONNECT_MESSAGE = "!DISCONNECT"

class Robot:
    def __init__(self,id,init_status,init_x,init_y):
        self.id = id
        self.current_x = init_x
        self.current_y = init_y
        self.status = init_status #Free: 0 || MovingTo: 1 || Lifting 2
        self.path = []
    def sendMessage(self):
        msg = str(self.id)+"|"+str(self.status)+"|"+str(self.current_x)+"|"+str(self.current_y)
        print(msg)
        send(msg)

    def receivePath(self):
        temp = client.recv(2048).decode('utf-8')
        print(temp)
        temp = temp.split(',')
        path = []
        for step in temp:
            num = int(step[:-1])
            dir = step[-1]
            path.append((num,dir))
        if(len(path)):
            self.path = path

    def move(self,step):
        num,dir = step
        print(num,dir)
        while num:
            if dir == "U" :
                self.current_y += 1
                self.status = 1
            elif dir == "D" :
                self.current_y += 1
                self.status = 1
            elif dir == "L" :
                self.current_x -= 1
                self.status = 1
            elif dir == "R" :
                self.current_x += 1
                self.status = 1
            elif dir == "P" :
                self.status = 2
                
            num -= 1
            self.sendMessage()
            time.sleep(0.5)
    def mainProcess(self):
        if(len(self.path)==0):
            self.status = 0
            self.sendMessage()
            self.receivePath()
            time.sleep(0.5)
        else:
            self.move(self.path[0])
            self.path.pop(0)
            print(self.path)
            # time.sleep(1)


def send(msg):
    message = msg.encode('utf-8')
    msg_length = len(message)
    send_length = str(msg_length).encode('utf-8')
    send_length += b' ' * (HEADER-len(send_length))
    client.send(send_length)
    client.send(message)

def receive():
    temp = client.recv(1024).decode('utf-8')
    return temp


client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client.connect(ADR)

robot = Robot("R2",0,7,4)
# robot.path = [(3,"U"),(2,"L"),(1,"D")]

while True:
    robot.mainProcess()
    temp = input("Press q to quit: ")
    if(temp == "q"):
        break
    # while len(robot.path):

    # temp = input("Press q to quit: ")
    # if(temp == "q"):
send(DISCONNECT_MESSAGE)