#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <stdio.h>
#include <iostream>

#define TCP_CLIENT_BUF_LEN 10240

using namespace std;

class TcpClient 
{
public:
    TcpClient(std::string ip, int port, bool is_block = true);
    ~TcpClient();

    bool inetSocket();
    bool inetConnect();
    bool isConnected();
    void inetClose();
    void getSocket(int &socket);

    bool inetSend(uint8_t* buf, int buf_len);
    int inetRecv(uint8_t* buf, int buf_len);

    bool inetSend(string send_msg);
    bool inetRecv(string& recv_msg);

private:
    int socket_;
    std::string ip_;
    int port_;
    struct sockaddr_in addr_;
    bool connected_;
    bool is_block_;

    bool connectByBlock();
    bool connectByNonBlock();
};

#endif