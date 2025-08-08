#!/usr/bin/env python


import socket
import time 

HOST = "172.16.15.10" #UR机器人的IP地址
PORT = 63352 #robotiq使用的端口
robotiq = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
robotiq.connect((HOST,PORT))
print("succeed!")

def gripperMove(position = 0, speed = 50, force = 50):
    """
    激活并控制夹爪的状态。
    position: 张开位置（0-255），默认为0
    speed: 速度（0-255），默认为50
    force: 力度（0-255），默认为50
    """
    robotiq.sendall(b'SET ACT 1\n') #完成激活
    robotiq.sendall(b'SET GTO 0\n') #重置末端动作
    robotiq.sendall(b'SET MOD 1\n') #设置优先模式
    robotiq.sendall(f'SET POS {position}\n'.encode())  # 设置张开位置
    robotiq.sendall(f'SET SPE {speed}\n'.encode())  # 设置速度
    robotiq.sendall(f'SET FOR {force}\n'.encode())  # 设置力度
    robotiq.sendall(b'SET GTO 1\n') #进行运动 
    print(f"Pos={position}, Speed={speed}, Force={force}")

    time.sleep(2) 

def getPosition():
    robotiq.sendall(b'GET POS\n') #获取当前位置
    data = robotiq.recv(2**10) #获取数据并转换二进制
    print("当前位置为：",robotiq)


if __name__ == "__main__":
    # gripperMove(70, 5, 100)
    gripperMove(100, 50, 0)
  
    robotiq.close()