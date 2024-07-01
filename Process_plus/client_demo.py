import socket
import time
import threading
# import serial
import time
import struct
import math
from time import sleep
HOST = "192.168.1.166"
# HOST = "10.89.234.39"
PORT = 2113
ADR = (HOST,PORT)

HEADER = 64
DISCONNECT_MESSAGE = "!DISCONNECT"

t_total = 1
T = 0.01
velX = 0
VelY = 0
qt_dot_x = 0
qt_dot_y = 0

z_status = 0
z_status_cur = 0
class Robot:
    def __init__(self,id,init_status,init_x,init_y):
        self.id = id
        self.current_x = init_x
        
        self.current_y = init_y
        self.status = init_status #Free: 0 || MovingTo: 1 || Lifting 2
        self.cur_velocity_x = 0
        self.cur_velocity_y = 0
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
    #def trans_uart_x(self ):
        
    def Trapezoidal_Velocity_X(Target_pos, current_position):
        global qt_dot_x
        global t_total
        global T
        
        A = 4 * (Target_pos - current_position) / (t_total * t_total)
        t1 = (t_total / 2) - 0.5 * math.sqrt((t_total * t_total) - (4 * abs(Target_pos - current_position) / A))

        for i in range(int(t_total / T) + 1):
            t = i * T
            if t <= t1 and t >= 0:
                qt = current_position + 0.5 * A * t * t
                qt_dot_x = A * t
            elif t > t1 and t <= t_total - t1:
                qt = current_position + A * t1 * (t - t1 / 2)
                qt_dot_x = A * t1
            elif t > (t_total - t1) and t <= t_total:
                qt = Target_pos - 0.5 * A * (t - t_total) * (t - t_total)
                qt_dot_x = A * (t_total - t)

            print(f"Time: {t:.2f}, Position: {qt:.2f}, Velocity: {qt_dot_x:.2f}")

    def Trapezoidal_Velocity_Y(Target_pos, current_position):
        global qt_dot_y
        global t_total
        global T
        
        A = 4 * (Target_pos - current_position) / (t_total * t_total)
        t1 = (t_total / 2) - 0.5 * math.sqrt((t_total * t_total) - (4 * abs(Target_pos - current_position) / A))

        for i in range(int(t_total / T) + 1):
            t = i * T
            if t <= t1 and t >= 0:
                qt = current_position + 0.5 * A * t * t
                qt_dot_y = A * t
            elif t > t1 and t <= t_total - t1:
                qt = current_position + A * t1 * (t - t1 / 2)
                qt_dot_y = A * t1
            elif t > (t_total - t1) and t <= t_total:
                qt = Target_pos - 0.5 * A * (t - t_total) * (t - t_total)
                qt_dot_y = A * (t_total - t)

            print(f"Time: {t:.2f}, Position: {qt:.2f}, Velocity: {qt_dot_y:.2f}")

        
        ## lay gia tri tuyet doi cho distance
    def move(self,step):
        global z_status
        num,dir = step
        print(num,dir) ## 2D111
        while num:
            match dir:
                case "U":
                    z_status = 1
                    target = self.current_y
                    self.status = 1
                    self.Trapezoidal_Velocity_Y(self.current_y , self.current_y+1)
                    self.current_y += 1
                    #m2v_Y(1,1) ##TODO:          
                case "D":
                    z_status = 1
                    self.status = 1
                    self.Trapezoidal_Velocity_Y(self.current_y , self.current_y - 1)
                    self.current_y -= 1
                    #m2v_Y(1,1) ##TODO:     
                case "L":
                    self.status = 1
                    self.Trapezoidal_Velocity_X(self.current_x , self.current_x - 1)
                    self.current_x -= 1
                    #m2v_X(1,-1)
                case "R":
                    self.current_x += 1
                    self.status = 1
                    self.Trapezoidal_Velocity_X(self.current_x , self.current_x + 1)
                    #m2v_X(1,-1)
                case "P":
                    self.status = 2
            # if dir == "U" :
            #     self.current_y += 1
            #     self.status = 1
            # elif dir == "D" :
            #     self.current_y += 1
            #     self.status = 1
            # elif dir == "L" :
            #     self.current_x -= 1
            #     self.status = 1
            # elif dir == "R" :
            #     self.current_x += 1
            #     self.status = 1
            # elif dir == "P" :
            #     self.status = 2


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

robot = Robot("R1",0,2,3)
# robot.path = [(3,"U"),(2,"L"),(1,"D")]
def handlServer():
    while True:
        robot.mainProcess()
        temp = input("Press q to quit: ")
        if(temp == "q"):
            break
    # while len(robot.path):

    # temp = input("Press q to quit: ")
    # if(temp == "q"):

# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
# ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
# ser2 = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.1)

# def handleUart():
#     while True:
#             # print("Nhap input")
#             # trans = input()
#             send_uart_signal()
#             #send_number(10)
#             print("sended")
#             #time.sleep(delay)
# # Hàm gửi tín hiệu UART
# def send_uart_signal(signal):
#     global qt_dot_x
#     global qt_dot_y
#     global z_status
#     if(z_status != z_status_cur):
#         ser2.write(z_status.encode('utf-8'))
#         sleep(5)
#     ser.write(qt_dot_x.encode('utf-8'))
#     ser1.write(qt_dot_y.encode('utf-8'))
#     z_status_cur = z_status


# Đoạn code để đọc giá trị từ các nút GPIO và gửi tín hiệu UART tương ứng


def start():
    print(f"[LISTENING] Server is listening on {socket.gethostbyname(socket.gethostname())}")
    order_thread = threading.Thread(target=handlServer)
    order_thread.start()
    # uart_thread = threading.Thread(target=handleUart)
    # uart_thread.start()


start()
send(DISCONNECT_MESSAGE)