/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Messages.h"

#include "protocol/nmea_util.h"

std::string Message::hexDump(int upto) const
{
    if ( upto == 0 ) { upto = buffer.size(); }
    return NMEA::hexDump(buffer.data(), upto);
}

void Message::appendCheckSum()
{
    uint8_t crc = 0;

    for (char c : buffer) {
        if (c != '$' && c != '!')
            crc ^= static_cast<uint8_t>(c);
    }

    buffer.push_back('*');
    buffer.push_back(NMEA::hex[crc >> 4]);
    buffer.push_back(NMEA::hex[crc & 0x0f]);
    buffer += "\r\n";
}


//
// the message pool
//
MessagePool::MessagePool()
{
    // Preallocate the desired number of messages
    _buffers.reserve(MSG_POOL_SIZE);
    for (int i = 0; i < MSG_POOL_SIZE; ++i) {
        _buffers.emplace_back(new Message);
        _freeList.push(_buffers.back());
    }
    _mutex = xSemaphoreCreateMutex();
}
MessagePool::~MessagePool()
{
    SemaphoreHandle_t tmp = _mutex;
    _mutex = nullptr;
    for (int i = 0; i < MSG_POOL_SIZE; ++i) {
        delete _buffers[i];
    }
    vSemaphoreDelete(tmp);
}

// granted none nullptr return value
Message* MessagePool::getOne(bool enforce)
{
    Message* msg = nullptr;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    while ( _freeList.empty() )
    {
        xSemaphoreGive(_mutex);
        if ( !enforce ) {
            _nr_acqfails++;
            return nullptr;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }
    msg = _freeList.front();
    msg->busy = true;
    _freeList.pop();
    _nr_acquisition++;
    xSemaphoreGive(_mutex);
    return msg;
}

void MessagePool::recycleMsg(Message* msg) {
    msg->busy = false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _freeList.push(msg);
    xSemaphoreGive(_mutex);
}

int MessagePool::nrFree() const
{
    return _freeList.size();
}
int MessagePool::nrUsed() const
{
    return MSG_POOL_SIZE - _freeList.size();
}

