from datetime import datetime
import time
from socket import *
import sys

# Адрес центральной платы
host = '192.168.0.13'
host2 = '192.168.0.11'
port = 81
addr = (host, port)
addr2 = (host2, port)

tcp_socket = socket(AF_INET, SOCK_STREAM)
tcp_socket.connect(addr)

tcp_socket2 = socket(AF_INET, SOCK_STREAM)
tcp_socket2.connect(addr2)

#data = input('write to server: ')
#if not data:
#    tcp_socket.close()
#    sys.exit(1)

# encode - перекодирует введенные данные в байты, decode - обратно
#data = str.encode(data)

# Флаги для формирования сообщения
comandFlag = "C"
addresFlag = "A"
dataFlag = "D"
endFlag = "x"

# комманды
cmd1 = "1"
cmd2 = "2"
cmd3 = "5"

# адрес переменных
#Addr1 = "4"
#Addr2 = "3"
#Addr3 = "2"

# данные для записи
wdata1 = "1"
wdata2 = "1"
wdata3 = "0"

# data = int(data)
# собираем сообщение
data = comandFlag
data += cmd1
data += addresFlag
data += dataFlag
data += wdata1
data += endFlag

data += comandFlag
data += cmd2
data += addresFlag
data += dataFlag
data += wdata2
data += endFlag
'''
data += comandFlag
data += cmd3
data += addresFlag
data += dataFlag
data += wdata3
data += endFlag'''
start_time = time.time()

tcp_socket.send(data.encode())
#data = bytes.decode(data)
data_r = tcp_socket.recv(1024)

tcp_socket2.send(data.encode())
data_r2 = tcp_socket2.recv(1024)
#time.sleep(1)
print(time.time() - start_time)

print(data_r)
print(data_r2)
tcp_socket.close()
