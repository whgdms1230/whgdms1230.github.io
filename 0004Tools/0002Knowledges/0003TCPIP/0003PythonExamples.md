---
sort: 3
---

# Python Examples

## 참고 문헌

*- [파이썬: TCP/IP 소켓 통신](https://duri1994.github.io/python/python-socket-network/)*

##### 서버 예제

```python
import socket, threading
import argparse

class Socket(object):
    def __init__(self, address='localhost', port=3333): # 서버 컴퓨터의 ip(여기선 내 컴퓨터를 서버 컴퓨터로 사용)
        # 서버소켓 오픈(대문을 열어둠)
        # socket.AF_INET: 주소종류 지정(IP) / socket.SOCK_STREAM: 통신종류 지정(UDP, TCP)
        # SOCK_STREAM은 TCP를 쓰겠다는 의미
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 여러번 ip.port를 바인드하면 에러가 나므로, 이를 방지하기 위한 설정이 필요
        # socket.SOL_SOCKET: 소켓 옵션
        # SO_REUSEADDR 옵션은 현재 사용 중인 ip/포트번호를 재사용할 수 있다.
        # 커널이 소켓을 사용하는 중에도 계속해서 사용할 수 있다
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.conn = self.socket
        self.address = address
        self.port = port
        self.timeout = None;
        self.socket.settimeout(self.timeout)
        
        self._bind()

    def _send(self, msg):
        self.conn.sendall(msg.encode(encoding='utf-8'))
        # sent = 0
        # while sent < len(msg):
        #     sent += self.conn.send(msg[sent:])

    def _read(self):
        data = self.conn.recv(1024)
        return data

    def _close(self):
        self.socket.close()
        if self.socket is not self.conn:
            self.conn.close()

    def _bind(self):
        self.socket.bind((self.address, self.port))

    def _listen(self):
        self.socket.listen()

    def _accept(self):
        return self.socket.accept()

    def _accept_connection(self):
        print("socket connection")
        self.conn, addr = self._accept()
        print('connect new addr: ', addr)

def server_thread(soc):
    while True:
        data = soc._read()
        msg = data.decode()
        if msg != '':
            print('read msg:', msg)
            soc._send(msg)
        
        if msg == '/end':
            break

parser = argparse.ArgumentParser()
parser.add_argument('--address', type=str, default='localhost')
parser.add_argument('--port', type=int, default=3333)
args = parser.parse_args()

soc = Socket(args.address, args.port)
soc._listen()
while True:

    soc._accept_connection()

    th = threading.Thread(target=server_thread, args=(soc,))
    th.start()
```

##### 클라이언트 예제

```python
# 클라이언트
import socket, argparse

parser = argparse.ArgumentParser()
parser.add_argument('--address', type=str, default='localhost')
parser.add_argument('--port', type=int, default=3333)
args = parser.parse_args()

server_ip = args.address # 위에서 설정한 서버 ip
server_port = args.port # 위에서 설정한 서버 포트번호

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((server_ip, server_port))

# /end 입력될 때 까지 계속해서 서버에 패킷을 보냄
while True:
    msg = input('msg:') 
    socket.sendall(msg.encode(encoding='utf-8'))
    data = socket.recv(100)
    msg = data.decode() 
    print('echo msg:', msg)
    
    if msg == '/end':
        break

socket.close()
```