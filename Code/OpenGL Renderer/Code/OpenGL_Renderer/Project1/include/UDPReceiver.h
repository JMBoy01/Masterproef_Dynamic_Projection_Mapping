// UDPReceiver.h
#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <iostream>
#include <winsock2.h>

#include "UDPMessage.h"

#pragma comment(lib, "ws2_32.lib")

class UDPReceiver {
private:
    SOCKET sock;
    sockaddr_in serverAddr;
    bool running;

public:
    UDPReceiver(int port);
    void start();
    void stop();
    void receiveData(UDPMessage& msg);
};

#endif // UDPRECEIVER_H