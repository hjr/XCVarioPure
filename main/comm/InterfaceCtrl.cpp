/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "InterfaceCtrl.h"

#include "DataLink.h"
#include "logdefnone.h"

#include <mutex>

// 1..n relation from interface to data link layer
// Ability to set interface details through a common cotrol interface
// Add and remove data links to the interface

InterfaceCtrl::InterfaceCtrl(bool oto, bool dl_supp) :
    _one_to_one(oto),
    _dl_support(dl_supp)
{
}

InterfaceCtrl::~InterfaceCtrl()
{
    DeleteAllDataLinks();
}

// get/create data link for this port
DataLink* InterfaceCtrl::newDataLink(int port)
{
    if ( ! _dl_support ) {
        ESP_LOGW(FNAME, "Interface %s does not support data links!", getStringId());
        return nullptr;
    }
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    if ( _one_to_one ) {
        // Always reuse
        if ( _dlink.empty() ) {
            _dlink[port] = new DataLink(port, getId());
        }
        return _dlink.begin()->second;
    }
    else {
        // Should be a different port to all in the list, or reuse
        auto it = _dlink.find(port);
        if ( it != _dlink.end()) {
            return it->second;
        }
        DataLink *newdl = new DataLink(port, getId());
        _dlink[port] = newdl;
        return newdl;
    }
}

// precondition: dl not yet in the map
void InterfaceCtrl::addDataLink(DataLink *dl)
{
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    if ( _one_to_one ) {
        // Always replace
        if ( ! _dlink.empty() ) {
            DeleteAllDataLinksLocked();
        }
        _dlink[dl->getPort()] = dl;
    }
    else {
        auto it = _dlink.find(dl->getPort());
        if ( it != _dlink.end() ) {
            DataLink *tmp = it->second;
            it->second = dl;
            delete tmp;
        }
        else {
            _dlink[dl->getPort()] = dl;
        }
    }
}

// returns possibly a nullptr, when port is not found
DataLink *InterfaceCtrl::MoveDataLink(int port)
{
    DataLink *tmp = nullptr;
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    if ( _one_to_one ) {
        tmp = _dlink.begin()->second;
        _dlink.clear();
    }
    else {
        auto it = _dlink.find(port);
        if ( it != _dlink.end() ) {
            tmp = it->second;
            _dlink.erase(it);
        }
    }
    return tmp;
}

void InterfaceCtrl::DeleteDataLink(int port)
{
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    if ( _one_to_one ) {
        DeleteAllDataLinksLocked();
    }
    else {
        auto it = _dlink.find(port);
        if ( it != _dlink.end() ) {
            DataLink *tmp = it->second;
            _dlink.erase(it);
            delete tmp;
        }
    }
}

void InterfaceCtrl::DeleteAllDataLinks()
{
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    DeleteAllDataLinksLocked();
}

void InterfaceCtrl::startMonitoring(ItfTarget tgt)
{
    // all data links
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    for ( auto dl : _dlink ) {
        ESP_LOGI(FNAME, "Mdl %x<>%x", (unsigned)dl.second->getTarget().raw, (unsigned)tgt.raw );
        if ( dl.second->getTarget() == tgt ) {
            dl.second->setMonitor(true);
        }
        else {
            dl.second->setMonitor(false);
        }
    }
}
void InterfaceCtrl::stopMonitoring()
{
    // all data links
    std::lock_guard<SemaphoreMutex> lock(_dlink_mutex);
    for ( auto dl : _dlink ) {
        dl.second->setMonitor(false);
    }
}

void InterfaceCtrl::DeleteAllDataLinksLocked()
{
    for (auto &it : _dlink ) {
        delete it.second;
    }
    _dlink.clear();
}
