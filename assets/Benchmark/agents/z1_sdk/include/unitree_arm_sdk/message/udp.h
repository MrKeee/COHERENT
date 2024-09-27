#ifndef _UDP_H
#define _UDP_H

#include <arpa/inet.h>
#include <termios.h>
#include <string>
#include <string.h>
#include <vector>
#include "unitree_arm_sdk/utilities/loop.h"
#include <chrono>

namespace UNITREE_ARM {
enum class BlockYN{
    YES,
    NO
};

class IOPort{
public:
    IOPort(BlockYN blockYN, size_t recvLength, size_t timeOutUs){
        resetIO(blockYN, recvLength, timeOutUs);
    }
    virtual ~IOPort(){}
    virtual size_t send(uint8_t *sendMsg, size_t sendLength) = 0;
    virtual size_t recv(uint8_t *recvMsg, size_t recvLength) = 0;
    virtual size_t recv(uint8_t *recvMsg) = 0;
    void resetIO(BlockYN blockYN, size_t recvLength, size_t timeOutUs);
    bool isDisConnect;
protected:
    BlockYN _blockYN = BlockYN::NO;
    size_t _recvLength;
    timeval _timeout;
    timeval _timeoutSaved;
    uint16_t _isDisConnectCnt;
};


class UDPPort : public IOPort{
public:
    UDPPort(std::string toIP, uint toPort, uint ownPort, 
            size_t recvLength = 0,
            BlockYN blockYN = BlockYN::NO,
            size_t timeOutUs = 20000);
    ~UDPPort();
    size_t send(uint8_t *sendMsg, size_t sendMsgLength);
    size_t recv(uint8_t *recvMsg, size_t recvLength);
    size_t recv(uint8_t *recvMsg);
private:
    size_t _recvBlock(uint8_t *recvMsg, size_t recvLength);
    size_t _recvUnBlock(uint8_t *recvMsg, size_t recvLength);
    sockaddr_in _ownAddr, _toAddr, _fromAddr;
    socklen_t _sockaddrSize;
    int _sockfd;
    int _on = 1;
    size_t _sentLength;
    fd_set _rSet;
    std::chrono::steady_clock::time_point last_recv_time_;
};  
}


#endif