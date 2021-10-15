#include <errno.h>
#include <fcntl.h> 
#include <cstring>
#include <unistd.h>
#include <string>

#include "tcp_client.h"
using namespace std;

TcpClient::TcpClient(std::string ip, int port, bool is_block): is_block_(is_block)
{
    memset(&addr_, 0, sizeof(addr_));
    printf("TcpClient: ip: %s, port: %d\n", ip.c_str(), port);
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = inet_addr(ip.c_str());
    printf("TcpClient: ip: %s, port: %d\n", ip.c_str(), port);
    addr_.sin_port = htons(port);
    ip_ = ip;
    socket_ = -1;
    connected_ = false;
}

TcpClient::~TcpClient()
{
    close(socket_);
}

void TcpClient::getSocket(int &socket)
{
    socket = socket_;
}

bool TcpClient::inetSocket()
{
    socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_ < 0)
    {
        printf("TcpClient: create socket failed,\n");
        return false;
    }

    if (!is_block_)
    {
        int flags = fcntl(socket_, F_GETFL, 0); //获取文件的flags值。
	    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);   //设置成非阻塞模式；
    }

    if (inet_pton(AF_INET, ip_.c_str(), &addr_.sin_addr) < 0) {
        printf("TcpClient: inet_pon failed\n");
        return false;
    }

    struct timeval timeout = {1,0};
	if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval)) != 0)
	{
		printf("set accept timeout failed");
	}

    return true;
}

bool TcpClient::isConnected()
{
    socklen_t addr_length = static_cast<socklen_t>(sizeof(addr_));
    int ret = getsockname(socket_, (struct sockaddr*)&addr_, &addr_length);
    switch (ret)
    {
        case EBADF:
        {
            printf("TcpClient: Socket is not a valid file descriptor\n");
            connected_ = false;
            return false;
        }
        break;
        case EFAULT:
        {
            printf("TcpClient: The memory pointed to by sockaddr is not a valid process space\n");
            connected_ = false;
            return false;
        }
        break;   
        case EINVAL:
        {
            printf("TcpClient: sockaddr length is invalid and may be negative\n");
            connected_ = false;
            return false;
        }
        break; 
        case ENOBUFS:
        {
            printf("TcpClient: Insufficient system resources when performing operations\n");
            connected_ = false;
            return false;
        }
        break; 
        case ENOTCONN:
        {
            printf("TcpClient: The socket is not connected.\n");
            connected_ = false;
            return false;
        }
        break; 
        case ENOTSOCK:
        {
            printf("TcpClient: Socket does not describe socket.\n");
            connected_ = false;
            return false;
        }
        break; 
        default:;
    }

    connected_ = true;
    return true;
}
  
bool TcpClient::connectByBlock()
{
    if(connected_) return true;

    int err = connect(socket_, (struct sockaddr*)&addr_, sizeof(addr_));
    if(err < 0)
    {
        printf("TcpClient: connect failed: %s(%d)\n", strerror(errno), errno);
        return false;
    }

    connected_ = true;
    return true;
}

bool TcpClient::connectByNonBlock()
{
    if(connected_) return true;
    int ret = connect(socket_, (struct sockaddr*)&addr_, sizeof(addr_));

    if(ret >= 0)
    {
        printf("TcpClient: connected.\n");
        connected_ = true;
        return true;
    }
    printf("TcpClient: to connect : %d.......\n", ret);
    if(errno == EINPROGRESS)
    {
        printf("TcpClient: to connect 1.......\n");
        printf("TcpClient: connecting...%s:%d\n",  ip_.c_str(), addr_.sin_port);

        printf("TcpClient: to connect 2 ......\n");
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(socket_, &writefds);                 
        printf("TcpClient: to connect 3.......\n");
        struct timeval timeout;         
        timeout.tv_sec = 1; 
        timeout.tv_usec = 0;     
        printf("TcpClient: to connect 4 ......\n");
        ret = select(socket_ + 1, NULL, &writefds, NULL, &timeout);
        printf("TcpClient: to connect 5.......\n");
        if(ret < 0)
        {
            printf("TcpClient: select error: (%d)%s\n", errno, strerror(errno));
            return false;
        }
        
        if (ret == 0)                         
        {           
            printf("TcpClient: connection timeout: (%d)%s\n", errno, strerror(errno));    
            return false;
        }

        if(!FD_ISSET(socket_, &writefds))
        {
            printf("TcpClient: no events found!: (%d)%s\n", errno, strerror(errno));   
            return false;
        }
        
        int err = 0;
        socklen_t elen = sizeof(err);
        ret = getsockopt(socket_, SOL_SOCKET, SO_ERROR, (char *)&err, &elen);
        if(ret < 0)
        {
            printf("TcpClient: no events found!: (%d)%s\n", errno, strerror(errno)); 
            return false;
        }
        
        if(err != 0)
        {
            printf("TcpClient: connect failed with the error: (%d)%s\n", errno, strerror(errno)); 
            return false;
        }
        
        printf("TcpClient: connected.\n");
        connected_ = true;
        return true;
    }
    else 
    {
        printf("TcpClient: disconnected.\n");
        return false;
    }
    
    printf("TcpClient: connected: %d\n", connected_);
}



bool TcpClient::inetConnect()
{
    if (is_block_)
    {
        return connectByBlock();
    }

    return connectByNonBlock();
}

bool TcpClient::inetSend(uint8_t* buf, int buf_len)
{
    if (!is_block_)
    {
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(socket_, &writefds);                 

        struct timeval timeout;         
        timeout.tv_sec = 3; 
        timeout.tv_usec = 0;     

        int ret = 0;
        if ((ret = select(socket_ + 1, &writefds, NULL, NULL, &timeout)) <= 0 ) /* rset没有使用过，不用重新置为sockfd */
        {
            printf("TcpClient: receive time out or connect error, %d, %s\n", errno, strerror(errno));
            return ret;
        }
    }

    if(send(socket_, buf, buf_len, 0) < 0)
    {
        printf("TcpClient: send failed, %d, %s\n", errno, strerror(errno));
        return false;
    }

    return true;
}

int TcpClient::inetRecv(uint8_t* buf, int buf_len)
{
    if (!is_block_)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(socket_, &readfds);                 

        struct timeval timeout;         
        timeout.tv_sec = 3; 
        timeout.tv_usec = 0;

        int ret = 0;
        if ((ret = select(socket_ + 1, &readfds, NULL, NULL, &timeout)) <= 0 ) /* rset没有使用过，不用重新置为sockfd */
        {
            printf("TcpClient: receive time out or connect error, %d, %s\n", errno, strerror(errno));
            return ret;
        }
    }

    int recv_bytes = recv(socket_, buf, buf_len, 0);
    if( recv_bytes < 0)
    {
        printf("TcpClient: recv failed, %d, %s\n", errno, strerror(errno));
    }

    return recv_bytes; 
}

bool TcpClient::inetRecv(std::string& recv_msg)
{
    uint8_t buf[TCP_CLIENT_BUF_LEN];
    memset(buf, 0, TCP_CLIENT_BUF_LEN);
    int recv_bytes = recv(socket_, buf, TCP_CLIENT_BUF_LEN, 0);

    if(recv_bytes < 0)
    {
        printf("TcpClient: recv failed, %d, %s\n", errno, strerror(errno));
        return false;
    }

    char recv_temp_char[TCP_CLIENT_BUF_LEN];
    memcpy(recv_temp_char, (char*)buf, recv_bytes);
    recv_temp_char[recv_bytes] = '\n';

    string recv_temp_string = recv_temp_char;
    recv_msg = recv_temp_string;
    
    return true; 
}

bool TcpClient::inetSend(std::string send_msg)
{
    char* send_msg_temp_char = const_cast<char*>(send_msg.c_str());

    uint8_t buf[TCP_CLIENT_BUF_LEN];
    memset(buf, 0, TCP_CLIENT_BUF_LEN);
    memcpy(buf, (uint8_t*)send_msg_temp_char, send_msg.size());

    if(send(socket_, buf, send_msg.size(), 0) < 0)
    {
        printf("TcpClient: send failed, %d, %s\n", errno, strerror(errno));
        return false;
    }

    return true; 
}

void TcpClient::inetClose()
{
    close(socket_);
}


