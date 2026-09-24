/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "Mutex.h"

#include <cstdint>
#include <array>

class DataLink;
class SensorBase;

// All managed interfaces -> max 16 (setup uses 4 bits to store)
// Do not change sequence of the enum values as they are used in nvs permanent storage
typedef enum {
    NO_PHY = 0,
    CAN_BUS,
    S0_RS232,
    S1_RS232,
    S2_RS232,
    WIFI_APSTA,
    BT_SPP,
    BT_LE,
    OW_BUS,
    GPIO_PIN,
    XCVPROXY // todo not yet implemented
} InterfaceId;

// Interface target
union ItfTarget {
    struct {
        InterfaceId iid  : 4;
        int         port : 21;
    };
    uint32_t raw;

    // Convenience
    constexpr ItfTarget(InterfaceId iid, int port=0)
        : raw((static_cast<InterfaceId>(iid) & 0xf) | ((port & 0x1fffff) << 4)) {}
    constexpr ItfTarget(int r = 0) : raw(r) {}
    constexpr bool operator<(const ItfTarget& other) const { return raw < other.raw; }
    constexpr bool operator==(const ItfTarget& other) const { return raw == other.raw; }
    constexpr bool operator!=(const ItfTarget& other) const { return raw != other.raw; }
    constexpr bool matchNoPhy(const ItfTarget& other) const {
        return ( (iid == NO_PHY && port == 0)               // both wildcards
                || (iid == NO_PHY && port == other.port)    // interface as wildcard
                || (iid == other.iid && port == 0)          // port as wildcard
                || raw == other.raw );
    }
    // getter
    constexpr InterfaceId getItfId() const { return iid; }
    constexpr int getPort() const { return port; }
    // setter
    constexpr void setItfId(InterfaceId id) { iid = id; }
    constexpr void setPort(int p) { port = p; }
};

// DataLink entries for mapping an interface to its corresponding data links.
class DataLinks {
    struct Entry {
        int key;
        DataLink* link;
    };

    std::array<Entry, 5> _entries{};
    uint8_t _size = 0;
    mutable SemaphoreMutex _mutex;

public:
    Entry* find(int key);
    DataLink* findDL(int key);
    bool insert(int key, DataLink* link);
    DataLink*  erase(int key);
    void deleteAllDataLinks();
    size_t size() const { return _size; }
    Entry* begin() {return &_entries[0]; }
    Entry* end() {return &_entries[0] + _size; }
};


// ISO/OSI 1..n relation from interface to data link layer.
// Because in the given embedded context a 1..1 relation is in most of the 
// use cases enough, it can be locked down to just one data link.

class InterfaceCtrl
{
public:
    InterfaceCtrl(bool oto=false, bool dl_supp=true);
    virtual ~InterfaceCtrl();

    virtual InterfaceId getId() const { return NO_PHY; }
    virtual const char* getStringId() const { return ""; }
    // Choose from an enumerated set of configurations, -1: OFF; 0: as is; 1,2,3..: load a preset profile 
    virtual void ConfigureIntf(int cfg) = 0;
    // if blocked returns number of ms for next possible invocation, returned len reflect the sent bytes
    // a negative return value reflects other errors.
    virtual int Send(const char *msg, int &len, int port=0) = 0;
    virtual void notifySensorDelete(SensorBase *) {} // to be called by sensors on deletion, so that the interface can cleanup data
    DataLink* newDataLink(int port);
    void addDataLink(DataLink *dl);
    DataLink* MoveDataLink(int port);
    void DeleteDataLink(int port);
    void startMonitoring(ItfTarget tgt);
    void stopMonitoring();
    int getNrDLinks() const { return _dlink.size(); }
    int isOneToOne() const { return _one_to_one; }
    bool getTestOk() const { return _functional; }

protected:
    DataLinks _dlink;
    uint8_t _functional :1 = false; // to be flipped from self tests

private:
    uint8_t _one_to_one :1;
    uint8_t _dl_support :1;
};