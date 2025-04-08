// UDPMessage.h
#ifndef UDPMESSAGE_H
#define UDPMESSAGE_H

struct UDPMessage {
    char flag; // 'e' voor extrinsic, 'i' voor intrinsic, 'p' voor pose
    double data[16]; // Groot genoeg om een 4x4 matrix te bevatten, of een combinatie van parameters
};

#endif // UDPMESSAGE_H