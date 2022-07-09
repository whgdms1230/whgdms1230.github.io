---
sort: 2
---

# C++ Examples

# TCP/IP Server C++ code

```cpp
#include <iostream>
#include <cstring>
#include <vector>
#include <memory>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

class SocketServer
{
    struct sockaddr_in serv_addr;
    struct sockaddr_in clnt_addr;

    public:
        SocketServer(std::string address, uint16_t port) : address_(address), port_(port) {
            socket_ = socket(AF_INET, SOCK_STREAM, 0);
            clnt_addr_size_ = sizeof(clnt_addr);

            memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET; // IPv4
            serv_addr.sin_addr.s_addr = inet_addr(address.c_str()); // IP  
            serv_addr.sin_port = htons(port); // port

            _bind();
        }

        ~SocketServer(){
            close(client_);
            close(socket_);
        }

        void _bind() {
            bind(socket_, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
        }

        void _listen(int backlog) {
            listen(socket_, backlog);
        }

        int _accept() {
            client_ = accept(socket_, (struct sockaddr*)&clnt_addr, &clnt_addr_size_);
            return client_;
        }

        void _send(std::string msg) {
            const char* str = msg.c_str();
            std::cout << "send: " << str << ", " << msg << std::endl;
            write(client_, str, sizeof(str));
        }

        std::string _read() {
            char buf[5000];
            int recv_len = recv(client_, buf, 1024, 0);
            std::cout << "read: " << recv_len << std::endl;
            if(recv_len == -1)
                return "";

            std::string str;
            for(std::size_t i = 0; i < recv_len; i++){
                std::cout << "read: " << buf[i] << std::endl;
                str += buf[i];
            }
            return str;
        }

    private:
        int socket_;
        int client_;
        socklen_t clnt_addr_size_;

        std::string address_;
        uint16_t port_;

};

int main(){

    auto soc = std::make_unique<SocketServer>("127.0.0.1", 3333);
    soc->_listen(20);

    while(1){
        int s = soc->_accept();
        std::cout << "accept: " << s << std::endl;
        if(s != -1){
            while(1){
                std::string data = soc->_read();
                if(data != ""){
                    // std::cout << "read msg: " << data << std::endl;
                    soc->_send(data);
                }

                if(data == "/end")
                    break;
            }
        }
    }

    std::cout << "log4" << std::endl;
    return 0;
}
```