#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <stdio.h>
#include <iostream>

#define TCP_SERVER_BUF_LEN 10240

using namespace std;

class TcpServer
{
public:
    TcpServer(std::string ip, int port);
    ~TcpServer();

    bool inetSocket();
    bool inetConnect();
    bool isAccept();
    void inetClose();

    bool inetSend(uint8_t* buf, int buf_len);
    int inetRecv(uint8_t* buf, int buf_len);

private:
    int socket_;
    struct sockaddr_in addr_;
    bool connected_;
};
#endif