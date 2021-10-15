#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <stdio.h>
#include <iostream>

#define UDP_CLIENT_BUF_LEN 10240

using namespace std;

class UdpClient 
{
public:
    UdpClient(std::string ip, int port);
    ~UdpClient();

    bool inetSocket();
    void inetClose();

    bool inetSend(uint8_t* buf, int buf_len);
    int inetRecv(uint8_t* buf, int buf_len);

private:
    int socket_;
    std::string ip_;
    int port_;
    struct sockaddr_in addr_;
};

#endif