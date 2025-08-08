# from ur_dashboard import DashboardClient
# robot_ip = "192.168.1.1"  # 替换为您的 UR5e IP
# client = DashboardClient(robot_ip)
# client.connect()
# client.close_popup()  # 关闭可能的弹窗
# client.unlock_protective_stop()  # 解除保护性停止（如果有）
# client.quit()  # 断开 Dashboard 连接

import socket

HOST = "172.16.15.10"
PORT = 502

try:
    robotiq = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robotiq.settimeout(2)
    robotiq.connect((HOST, PORT))
    print("连接成功")

    # 读取输入寄存器0x03E8，长度6
    read_cmd = b'\x00\x01\x00\x00\x00\x06\x09\x04\x03\xe8\x00\x06'
    robotiq.send(read_cmd)
    data = robotiq.recv(1024)
    print("收到数据:", data)
except Exception as e:
    print("发生错误:", e)
finally:
    robotiq.close()
# 运行此脚本前，请确保Robotiq Hand-E夹爪已连接
