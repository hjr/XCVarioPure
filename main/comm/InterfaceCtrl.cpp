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

// DataLinks
DataLinks::Entry* DataLinks::find(int key)
{
    for (uint8_t i = 0; i < _size; ++i) {
        if (_entries[i].link && _entries[i].key == key) {
            return &_entries[i];
        }
    }
    return nullptr;
}
DataLink* DataLinks::findDL(int key)
{
    for (uint8_t i = 0; i < _size; ++i) {
        if (_entries[i].link && _entries[i].key == key) {
            return _entries[i].link;
        }
    }
    return nullptr;
}
bool DataLinks::insert(int key, DataLink* link)
{
    std::lock_guard<SemaphoreMutex> lock(_mutex);
    
    if (_size >= _entries.size()) {
        return false;
    }

    Entry& it = _entries[_size];
    for (uint8_t i = 0; i < _size; ++i) {
        if (_entries[i].link && _entries[i].key == key) {
            it = _entries[i];
            break;
        }
    }
    if ( it.link ) {
        // reuse
        DataLink* tmp = it.link;
        it.link = nullptr;
        delete tmp;
        it.link = link;
    }
    else {
        // add
        it.key = key;
        it.link = link;
        _size++;
    }
    return true;
}
DataLink* DataLinks::erase(int key)
{
    std::lock_guard<SemaphoreMutex> lock(_mutex);

    DataLink* erasedLink = nullptr;
    for (uint8_t i = 0; i < _size; ++i) {
        if (_entries[i].key == key) {

            erasedLink = _entries[i].link;
            _entries[i].key = 0;
            _entries[i].link = nullptr;
            if ( i != _size - 1 ) {
                _entries[i] = _entries[_size - 1];
            }
            --_size;
        }
    }
    return erasedLink;
}
void DataLinks::deleteAllDataLinks()
{
    std::lock_guard<SemaphoreMutex> lock(_mutex);

    for (auto &it : _entries ) {
        DataLink* tmp = it.link;
        it.link = nullptr;
        it.key = 0;
        if (tmp) delete tmp;
    }
    _size = 0;
}

// InterfaceCtrl
//
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
    _dlink.deleteAllDataLinks();
}

// get/create data link for this port
DataLink* InterfaceCtrl::newDataLink(int port)
{
    if ( ! _dl_support ) {
        ESP_LOGW(FNAME, "Interface %s does not support data links!", getStringId());
        return nullptr;
    }
    if ( _one_to_one ) {
        // Create one, or reuse existing
        if ( _dlink.size() == 0 ) {
            _dlink.insert(port, new DataLink(port, getId()));
        }
        return _dlink.begin()->link;
    }
    else {
        // Should be a different port to all in the list, or reuse
        DataLink *dl = _dlink.findDL(port);
        if ( dl ) {
            return dl;
        }
        DataLink *newdl = new DataLink(port, getId());
        if (_dlink.insert(port, newdl)) {
            return newdl;
        }
        delete newdl;
        return nullptr;
    }
}

// precondition: dl not yet in the map
void InterfaceCtrl::addDataLink(DataLink *dl)
{
    if ( _one_to_one ) {
        // Always replace
        if ( _dlink.size() ) {
            _dlink.deleteAllDataLinks();
        }
        _dlink.insert(dl->getPort(), dl);
    }
    else {
        // option to replace an existing entry if needed
        _dlink.insert(dl->getPort(), dl);
    }
}

// returns possibly a nullptr, when port is not found in the map
DataLink *InterfaceCtrl::MoveDataLink(int port)
{
    return _dlink.erase(port);
}

void InterfaceCtrl::DeleteDataLink(int port)
{
    DataLink *tmp = _dlink.erase(port);
    if ( tmp ) {
        delete tmp;
    }
}

void InterfaceCtrl::startMonitoring(ItfTarget tgt)
{
    // all data links
    for ( auto it = _dlink.begin(); it != _dlink.end(); ++it ) {
        auto &dl = *it;
        ESP_LOGI(FNAME, "Mdl %x<>%x", (unsigned)dl.link->getTarget().raw, (unsigned)tgt.raw );
        if ( dl.link->getTarget() == tgt ) {
            dl.link->setMonitor(true);
        }
        else {
            dl.link->setMonitor(false);
        }
    }
}
void InterfaceCtrl::stopMonitoring()
{
    // all data links
    for ( auto it = _dlink.begin(); it != _dlink.end(); ++it ) {
        it->link->setMonitor(false);
    }
}

