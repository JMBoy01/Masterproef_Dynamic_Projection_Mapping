#include <iostream>
#include <winsock2.h>

#include "UDPReceiver.h"

#pragma comment(lib, "ws2_32.lib")

UDPReceiver::UDPReceiver(int port) : running(false) {
    WSAData wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed!" << std::endl;
        exit(1);
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed!" << std::endl;
        WSACleanup();
        exit(1);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);
}

void UDPReceiver::start() {
    if (bind(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Binding failed!" << std::endl;
        closesocket(sock);
        WSACleanup();
        exit(1);
    }
    running = true;
    std::cout << "UDP Receiver started." << std::endl;
}

void UDPReceiver::stop() {
    running = false;
    closesocket(sock);
    WSACleanup();
    std::cout << "UDP Receiver stopped." << std::endl;
}

void UDPReceiver::receiveData(UDPMessage& msg) {
    if (!running) {
        std::cerr << "Receiver is not running!" << std::endl;
        return;
    }

    std::cout << "Receiver started receiving data!" << std::endl;

    sockaddr_in clientAddr;
    int clientLen = sizeof(clientAddr);

    while (running) {
        int bytesReceived = recvfrom(sock, reinterpret_cast<char*>(&msg), sizeof(UDPMessage), 0,
            (sockaddr*)&clientAddr, &clientLen);

        if (bytesReceived == SOCKET_ERROR) {
            std::cerr << "Receiving failed!" << std::endl;
            break;
        }

        // Debug print
        /*std::cout << "Received flag: " << msg.flag << std::endl;
        std::cout << "Received matrix: " << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << msg.data[i * 4] << " "
                << msg.data[i * 4 + 1] << " "
                << msg.data[i * 4 + 2] << " "
                << msg.data[i * 4 + 3] << std::endl;
        }
        std::cout << "------------------" << std::endl;*/
        std::cout << "Received: " << msg.flag << std::endl;
    }
}