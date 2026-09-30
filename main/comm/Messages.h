/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Configuration.h"

#include <string>
#include <queue>
#include <vector>


constexpr int MSG_POOL_SIZE = 6;
constexpr int PREALLOC_BUF_SIZE = 128;

// One Message
class Message
{
public:
    Message() { buffer.reserve(PREALLOC_BUF_SIZE); }
    void appendCheckSum();
    std::string hexDump(int upto=0) const;
    bool busy = false;
    DeviceId target_id = DeviceId::NO_DEVICE;
    int port = 0;
    std::string buffer;
};

// Preallocated pool of messages
class MessagePool {
public:
    MessagePool();
    ~MessagePool();

    Message* getOne(bool enforce=true);
    void recycleMsg(Message* msg);

    // Some debug and statistics
    int nrAcqFails() const { return _nr_acqfails; }
    int nrFree() const;
    int nrUsed() const;
    int nrAcq() const { return _nr_acquisition; }

private:
    std::vector<Message *> _buffers;
    std::queue<Message *> _freeList;
    long _nr_acquisition = 0;
    long _nr_acqfails = 0;
    SemaphoreHandle_t _mutex = nullptr;
};



namespace DEV
{

Message* plsMessage(RoutingTarget target); // can fail
Message* acqMessage(DeviceId target, int port); // garanty, but potential wait
inline void relMessage(Message *msg);

bool Send(Message* msg);

}