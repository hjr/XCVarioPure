/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "DeviceMgr.h"

#include "Configuration.h"
#include "Messages.h"
#include "CanBus.h"
#include "WifiApSta.h"
#include "BTspp.h"
#include "BTnus.h"
#include "OneWireBus.h"
#include "SerialLine.h"
#include "protocol/ProtocolItf.h"
#include "protocol/NMEA.h"
#include "protocol/MagSensBin.h"
#include "protocol/CANPeerCaps.h"
#include "setup/DataMonitor.h"
#include "setup/SetupNG.h"
#include "Compass.h"
#include "sensor/temp/OwSens.h"
#include "sensor/gps/GpsVSensor.h"
#include "sensor/temp/TempVSens.h"

#include "sensor.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <deque>
#include <mutex>

// global variables
DeviceManager* DEVMAN = nullptr; // singleton like
QueueHandle_t ItfSendQueue = nullptr;
MessagePool MP;

// static vars
static TaskHandle_t SendTask = nullptr;

//
// A routing table that contains the static connected devices relation.
//
// Target lists
// target entries with zero termination, as ro flash data
static const RoutingTarget flarm_routes[] = {
    {FLARM_HOST_DEV, S2_RS232, 0}, {FLARM_HOST_DEV, WIFI_APSTA, 8881}, {FLARM_HOST_DEV, BT_SPP, 0}, {FLARM_HOST_DEV, BT_LE, 0}, 
    {XCVARIOSECOND_DEV, CAN_BUS, 0}, {XCVARIOFIRST_DEV, CAN_BUS, 0}, {XCVARIOSECOND_DEV, WIFI_APSTA, 8884}, {XCVARIOFIRST_DEV, WIFI_APSTA, 8884},
    {FLARM_HOST2_DEV, WIFI_APSTA, 8881}, {FLARM_HOST2_DEV, BT_SPP, 0}, {FLARM_HOST2_DEV, BT_LE, 0}, {} };
static const RoutingTarget flarm_proxy_routes[] = {
    {FLARM_HOST_DEV, S2_RS232, 0}, {FLARM_HOST_DEV, WIFI_APSTA, 8881}, {FLARM_HOST_DEV, BT_SPP, 0}, {FLARM_HOST_DEV, BT_LE, 0}, 
    {FLARM_HOST2_DEV, WIFI_APSTA, 8881}, {FLARM_HOST2_DEV, BT_SPP, 0}, {FLARM_HOST2_DEV, BT_LE, 0}, {} };
static const RoutingTarget radio_routes[] = {
    {RADIO_KRT2_DEV, S2_RS232, 0}, {RADIO_ATR833_DEV, S2_RS232, 0}, {XCVARIOFIRST_DEV, CAN_BUS, 0}, {} };
static const RoutingTarget navi_routes[] = {
    {FLARM_DEV, S1_RS232, 0}, {FLARM_DEV, CAN_BUS, 0}, {RADIO_KRT2_DEV, S2_RS232, 0}, {} };
static const RoutingTarget fhost_routes[] = {
    {FLARM_DEV, S1_RS232, 0}, {FLARM_DEV, CAN_BUS, 0}, {} };
static const RoutingTarget xcv_proxy_routes[] = {
    {NAVI_DEV, S2_RS232, 0}, {FLARM_HOST_DEV, WIFI_APSTA, 8881}, {FLARM_HOST_DEV, BT_SPP, 0}, {FLARM_HOST_DEV, BT_LE, 0}, {} };
// Mapping table with (one hop): From-Target <> To-Target-List
static const std::pair<RoutingTarget, const RoutingTarget*> Routes[] = {
    { RoutingTarget(FLARM_DEV, S1_RS232, 0), flarm_routes },
    { RoutingTarget(FLARM_DEV, S2_RS232, 0), flarm_routes },
    { RoutingTarget(FLARM_DEV, BT_SPP, 0), flarm_routes },
    { RoutingTarget(FLARM_DEV, CAN_BUS, 0), flarm_proxy_routes },
    { RoutingTarget(RADIO_REMOTE_DEV, NO_PHY, 0), radio_routes },
    { RoutingTarget(NAVI_DEV, NO_PHY, 0), navi_routes },
    { RoutingTarget(FLARM_HOST_DEV, NO_PHY, 0), fhost_routes },
    { RoutingTarget(FLARM_HOST2_DEV, NO_PHY, 0), fhost_routes },
    { RoutingTarget(XCVARIOFIRST_DEV, NO_PHY, 0), xcv_proxy_routes },
    { RoutingTarget(XCVARIOSECOND_DEV, NO_PHY, 0), xcv_proxy_routes }
};
// Search the flash data table
static const RoutingTarget* findRoute(const RoutingTarget& source) {
    // Search through Routes and find a match to the source device
    for (const auto& entry : Routes) {
        if (entry.first.match(source)) {
            return entry.second; // Return the matched routing list
        }
    }
    return nullptr;
}

//
// Known device attributes
//
// Define device names for the configuration menu
// - The device id, must be a unique entry in this list with associated name
// - Device attributes
//   + a readable name
//   + a list of interfaces, first one is a default proposal
//   + later entries with same dev id speicialize and override the first default, but need to be there. 
//     -> Addition per setup needs to select the remaining attributes through the paired (device id + interface)
//   + a list of protocol, first N are the by default configured ones
//   + an interface profile enum, or port, if needed
//   + device flags
//   + optional pointer to a SetupNG<> configuration entry for this device
//
constexpr std::pair<DeviceId, DeviceAttributes> DEVATTR[] = {
    {DeviceId::ANEMOI_DEV, {"Anemoi", {{S1_RS232, S2_RS232}}, {{ANEMOI_P}, 1}, 0, IS_SEL, &anemoi_devsetup}},
    {DeviceId::ANEMOI_DEV, {"", {{S2_RS232}}, {{ANEMOI_P}, 1}, 0, 0, nullptr}},
    {DeviceId::CANREGISTRAR_DEV, {"Auto-connect", {{CAN_BUS}}, {{REGISTRATION_P}, 1}, CAN_REG_PORT, 0, nullptr}},
    {DeviceId::FLARM_DEV,  {"Flarm", {{S1_RS232, S2_RS232, BT_SPP}}, {{FLARM_P, FLARMBIN_P}, 2}, 0, IS_SEL, &flarm_devsetup}},
    {DeviceId::FLARM_DEV,  {"", {{S2_RS232}}, {{FLARM_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::FLARM_DEV,  {"", {{BT_SPP}}, {{FLARM_P}, 1}, 0, 0, nullptr}},
    // {DeviceId::FLARM_DEV,  {"", {{XCVPROXY}}, {{FLARM_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::JUMBO_DEV,  {"jumbo putzi", {{CAN_BUS}}, {{JUMBOCMD_P}, 1} , 0, 0, nullptr}}, // auto reg
    {DeviceId::XCVARIOFIRST_DEV, {"Master XCV", {{WIFI_APSTA, S2_RS232}}, {{XCVSYNC_P}, 1}, 8884, IS_SEL|SECOND_ONLY, &master_devsetup}}, // CAN_BUS auto reg
    // {DeviceId::XCVARIOFIRST_DEV, {"", {{BT_SPP}}, {{XCVSYNC_P}, 1}, 0, 0, nullptr}},
    {DeviceId::XCVARIOFIRST_DEV, {"", {{S2_RS232}}, {{XCVSYNC_P}, 1}, 0, 0, &master_devsetup}},
    {DeviceId::XCVARIOSECOND_DEV, {"Second XCV", {{WIFI_APSTA, S2_RS232}}, {{XCVSYNC_P}, 1}, 8884, IS_SEL|MASTER_ONLY, &second_devsetup}}, // CAN_BUS auto reg
    // {DeviceId::XCVARIOSECOND_DEV, {"", {{BT_SPP}}, {{XCVSYNC_P}, 1}, 0, 0, nullptr}}, todo, missing the BLUEspp client implementation
    {DeviceId::XCVARIOSECOND_DEV, {"", {{S2_RS232}}, {{XCVSYNC_P}, 1}, 0, 0, nullptr}},
    {DeviceId::MAGLEG_DEV, {"MagSens rev0", {{CAN_BUS}}, {{MAGSENSBIN_P}, 1}, MagSensBin::LEGACY_MAGSTREAM_ID, IS_SEL, &magleg_devsetup}},
    {DeviceId::MAGSENS_DEV, {"MagSens rev1", {{CAN_BUS}}, {{MAGSENS_P}, 1}, 0, IS_SEL, nullptr}}, // auto reg
    {DeviceId::NAVI_DEV,   {"Navi", {{WIFI_APSTA, S1_RS232, S2_RS232, BT_SPP, BT_LE, CAN_BUS}},
                                    {{XCVARIO_P, CAMBRIDGE_P, OPENVARIO_P, BORGELT_P, SEEYOU_P, KRT2_REMOTE_P, ATR833_REMOTE_P}, 1},
                                    8880, IS_SEL, &navi_devsetup}},
    {DeviceId::NAVI_DEV,   {"", {{S2_RS232}}, {{XCVARIO_P, CAMBRIDGE_P, OPENVARIO_P, BORGELT_P, KRT2_REMOTE_P, ATR833_REMOTE_P}, 1},
                                    0, 0, nullptr}},
    {DeviceId::NAVI_DEV,   {"", {{BT_SPP}}, {{XCVARIO_P, CAMBRIDGE_P, OPENVARIO_P, BORGELT_P, KRT2_REMOTE_P, ATR833_REMOTE_P}, 1},
                                    0, 0, nullptr}},
    {DeviceId::NAVI_DEV,   {"", {{BT_LE}}, {{XCVARIO_P, CAMBRIDGE_P, OPENVARIO_P, BORGELT_P, SEEYOU_P, KRT2_REMOTE_P, ATR833_REMOTE_P}, 1},
                                    0, 0, nullptr}},
    {DeviceId::FLARM_HOST_DEV, {"Flarm Consumer", {{WIFI_APSTA, S2_RS232, BT_SPP, BT_LE}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 8881, IS_SEL, &flarm_host_setup}},
    {DeviceId::FLARM_HOST_DEV, {"", {{S2_RS232}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::FLARM_HOST_DEV, {"", {{BT_SPP}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::FLARM_HOST_DEV, {"", {{BT_LE}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::FLARM_HOST2_DEV, {"Flarm Download", {{WIFI_APSTA, BT_SPP}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 8881, IS_SEL, &flarm_host2_setup}},
    {DeviceId::FLARM_HOST2_DEV, {"", {{BT_SPP}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::FLARM_HOST2_DEV, {"", {{BT_LE}}, {{FLARMHOST_P, FLARMBIN_P}, 2}, 0, 0, nullptr}},
    {DeviceId::RADIO_REMOTE_DEV, {"Radio remote", {{WIFI_APSTA}}, {{KRT2_REMOTE_P}, 1}, 8882, IS_SEL, &radio_host_setup}},
    {DeviceId::RADIO_KRT2_DEV, {"KRT 2", {{S2_RS232, CAN_BUS}}, {{KRT2_REMOTE_P}, 1}, 0, IS_SEL, &krt_devsetup}},
    {DeviceId::RADIO_ATR833_DEV, {"ATR833", {{S2_RS232, CAN_BUS}}, {{ATR833_REMOTE_P}, 1}, 0, IS_SEL, &atr_devsetup}},
    {DeviceId::TEMPSENS_DEV, {"Temp. Sensor", {{OW_BUS}}, {{NO_ONE}, 0}, 0, IS_SEL, nullptr}},
};

// Lookup functions
constexpr const DeviceAttributes NullDev("Null", {{NO_PHY}}, {{NO_ONE}}, 0, 0, nullptr);
const DeviceAttributes& DeviceManager::getDevAttr(DeviceId did, InterfaceId via) {
    // retrieve first attribute entry
    for (const auto& entry : DEVATTR) {
        if (entry.first == did) {
            if (via == NO_PHY) {
                return entry.second;
            }
            if (via == entry.second.itfs.get(0)) {
                return entry.second;
            }
        }
    }
    return NullDev;
}
std::string_view DeviceManager::getDevName(DeviceId did) {
    auto da = getDevAttr(did);
    if ( &da != &NullDev) {
        return da.name; // first hit rules
    }
    return "";
}
std::vector<DeviceId> DeviceManager::allKnownDevs()
{
    std::vector<DeviceId> ret;
    for (const auto& entry : DEVATTR) {
        if (! entry.second.name.empty()  && ! entry.second.isVariant()) { // fixme empty()?
            ret.push_back(entry.first);
        }
    }
    return ret;
}

constexpr std::pair<InterfaceId, std::string_view> INTFCS[] = {
    {CAN_BUS, "CAN bus"},
    {OW_BUS, "OneWire bus"},
    {S1_RS232, "S1 serial"},
    {S2_RS232, "S2 serial"},
    {WIFI_APSTA, "Wifi"},
    {BT_SPP, "BT serial"},
    {BT_LE, "BT low energy"}
    // {XCVPROXY, "XCV proxy"}
};

std::string_view DeviceManager::getItfName(InterfaceId iid) {
    for (const auto& entry : INTFCS) {
        if (entry.first == iid) {
            return entry.second;
        }
    }
    return "";
}

std::vector<InterfaceId> DeviceManager::allKnownIntfs()
{
    std::vector<InterfaceId> ret;
    for (const auto& entry : INTFCS) {
        ret.push_back(entry.first);
    }
    return ret;
}

constexpr std::pair<ProtocolType, std::string_view> PRTCLS[] = {
    {REGISTRATION_P, "Auto Registration"},
    {XCVSYNC_P, "XCV Sync"},
    {JUMBOCMD_P, "jumbo"},
    {ANEMOI_P, "Anemoi"},
    {FLARM_P, "Flarm"},
    {FLARMHOST_P, "Flarm Consumer"},
    {FLARMBIN_P, "Flarm Download"},
    {GARMIN_P, "Garmin"},
    {MAGSENS_P, "Magsens"},
    {MAGSENSBIN_P, "Magsens Data"},
    {NMEASTD_P, "NMEA Std"},
    {XCVARIO_P, "XCVario"},
    {OPENVARIO_P, "Open-Vario"},
    {BORGELT_P, "Borgelt"},
    {CAMBRIDGE_P, "Cambridge"},
    {SEEYOU_P, "SeeYou"},
    {KRT2_REMOTE_P, "KRT2"},
    {ATR833_REMOTE_P, "ATR833"},
    {XCVQUERY_P, "XCV Query"},
};

std::string_view DeviceManager::getPrtclName(ProtocolType pid) {
    for (const auto& entry : PRTCLS) {
        if (entry.first == pid) {
            return entry.second;
        }
    }
    return "";
}

// a dummy interface
class DmyItf final : public InterfaceCtrl
{
public:
    DmyItf() : InterfaceCtrl(true, false) {}
    const char* getStringId() const override { return "NUL"; }
    void ConfigureIntf(int cfg) override {}
    int Send(const char *msg, int &len, int port=0) { return 0; }
};
static DmyItf dummy_itf;
static ItfTarget monitor_target = {};

static int tt_snd(Message *msg)
{
    int len = msg->buffer.length();
    int port = msg->port;
    InterfaceCtrl *itf = DEVMAN->getIntf(msg->target_id);
    int plsrety = 0;
    if (itf)
    {
        ESP_LOGD(FNAME, "send %s/%d NMEA len %d, msg: %s", itf->getStringId(), port, len, msg->buffer.c_str());
        plsrety = itf->Send(msg->buffer.c_str(), len, port);
        if ( plsrety > 0 ) {
            ESP_LOGD(FNAME, "reshedule message %d/%d", len, msg->buffer.length());
            msg->buffer.erase(0, len); // chop sent bytes off
        }
        else if ( DM && plsrety == 0 && monitor_target == ItfTarget(itf->getId(), port) ) {
            DM->monitorString(DIR_TX, !std::isprint(msg->buffer[0]), msg->buffer.c_str(), len);
        }
    }
    return plsrety;
}

// generic transmitter grabbing messages from a queue
static void transmit_task(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    Message *msg;
    std::deque<Message *> later;
    int pls_retry = 0;

    while (true)
    {
        // sleep until the queue gives us something to do, or we have to do a retry
        TickType_t timeout = (pls_retry==0) ? portMAX_DELAY : pdMS_TO_TICKS(pls_retry);
        bool new_msg = xQueueReceive(queue, &msg, timeout) == pdTRUE;

        if ( new_msg ) {
            if (msg == nullptr) {
                break;
            } // termination signal

            // always check against the later fifo first, not to garble msg sequence
            bool wait = false;
            if( !later.empty() ) {
                // might be just another msg to wait for the same itf
                for ( Message *i : later ) {
                    if ( msg->target_id == i->target_id ) {
                        wait = true;
                        break;
                    }
                }
            }
            if ( wait ) {
                // put this straight onto the later queue
                ESP_LOGD(FNAME, "straight wait");
                later.push_back(msg);
            }
            else {
                // Regular case
                ESP_LOGD(FNAME, "regular send");
                pls_retry = tt_snd(msg);
                if ( pls_retry < 1 ) {
                    ESP_LOGD(FNAME, "regular done");
                    DEV::relMessage(msg);
                } else {
                    ESP_LOGD(FNAME, "retry pushed");
                    later.push_back(msg);
                    continue; // just pushed this one, no need to check again
                }
            }
        }

        // Always have to go through the later list until the first msg gets rejected
        // to know the proper tim-out to wait again
        while ( !later.empty() ) {
            msg = later.front();
            int cmplen = msg->buffer.size();
            ESP_LOGD(FNAME, "postponed send %d", cmplen);
            pls_retry = tt_snd(msg);
            if ( pls_retry > 0 ) {
                if ( (cmplen - msg->buffer.size()) > 0 ) {
                    break; // some bytes got sent
                }
                // Inaceptable for a retry, discard
                pls_retry = 0;
            }
            DEV::relMessage(msg);
            later.pop_front();
        }
    }
    vTaskDelete(NULL);
}

DeviceManager::DeviceManager()
{
    ItfSendQueue = xQueueCreate( MSG_POOL_SIZE+1, sizeof(Message*) );
}

DeviceManager::~DeviceManager()
{
    xQueueSend( ItfSendQueue, nullptr, 0 ); // terminate transmitter
}

// The device manager
DeviceManager* DeviceManager::Instance()
{
    if ( ! DEVMAN ) {
        DEVMAN = new DeviceManager();
    }
    return DEVMAN;
}

// Do all it needs to prepare comm with device and route data
// It returns the pointer to the device
Device* DeviceManager::addDevice(DeviceId did, ProtocolType proto, int listen_port, int send_port, InterfaceId iid, bool nvsave)
{
    // On first device a send task needs to be created
    if ( ! SendTask ) {
        xTaskCreate(transmit_task, "genTx", 3000, ItfSendQueue, 21, &SendTask);
    }
    ESP_LOGI(FNAME, "Add device %d via %d (lp %d, sp %d) proto %d", did, iid, listen_port, send_port, proto);
    InterfaceCtrl *itf = &dummy_itf;
    if ( iid == CAN_BUS ) {
        // CAN is special and not on all HW versions.
        // Do not try to create it
        if ( CAN && CAN->getTestOk() ) {
            if ( ! CAN->isInitialized() ) {
                CAN->ConfigureIntf(0); // default setup
            }
            itf = CAN;
        }
    }
    // else if ( iid == XCVPROXY) {
    //     itf = &dummy_itf;
    //     Device *dev = getXCVPeer();
    //     if ( dev ) {
    //         // XCV proxy is already there
    //         itf = dev->_itf;
    //         listen_port = dev->getListenPort();
    //         send_port = dev->getSendPort(XCVSYNC_P);
    //     }
    // }
    else if ( iid == WIFI_APSTA ) {
        if ( ! WIFI ) {
            WifiApSta::createWifiApSta();
        }
        if ( WIFI ) {
            WIFI->ConfigureIntf(listen_port);
            itf = WIFI;
        }
    }
    else if ( iid == S1_RS232) {
        if ( S1 ) { // S1 is instantiated allways (!)
            S1->ConfigureIntf(-1); // load nvs setup
            itf = S1;
        }
    }
    else if ( iid == S2_RS232) {
        if ( S2 ) { // S2 is instantiated allways (!)
            S2->ConfigureIntf(-1);
            itf = S2;
        }
    }
    else if ( iid == BT_SPP) {
        if ( ! BLUEspp ) {
            ESP_LOGI(FNAME, "Create BT SPP interface");
            BLUEspp = new BTspp();
        }
        if ( BLUEspp && ! BLUEspp->isRunning() ) {
            BLUEspp->start();
        }
        itf = BLUEspp;
    }
    else if ( iid == BT_LE) {
        if ( ! BLUEnus ) {
            ESP_LOGI(FNAME, "Create BT LE interface");
            BLUEnus = new BTnus();
        }
        if ( BLUEnus && ! BLUEnus->isRunning() ) {
            BLUEnus->ConfigureIntf(proto);
        }
        itf = BLUEnus;
    }
    else if ( iid == OW_BUS ) {
        if ( ! OneWIRE ) {
            itf = OneWireBus::create();
            itf->ConfigureIntf(0);
            ESP_LOGI(FNAME, "OneWire conf");
        }
        itf = OneWIRE;
    }
    // else // NO_PHY is just the hint to take the same interface

    bool is_new = false;
    Device *dev = getDevice(did);
    if (dev && itf == &dummy_itf) { // that means NO_PHY -> "take the same" was requeset
        // Device already exists
        itf = dev->_itf;
    }
    if ( dev && dev->_itf != itf ) {
        // Device already exists, but on another interface
        ESP_LOGW(FNAME, "Device %d already exists on another interface", did);
        return nullptr;
    }
    if ( ! dev ) {
        dev = new Device(did);
        is_new = true;
        // Retrieve, or create a new data link
        dev->_link = itf->newDataLink(listen_port);
        dev->_itf = itf;
        if ( dev->_link ) {
            dev->_link->incrDeviceCount();

            // for some devices we need to create some gears to process the data stream
            if ( did == MAGLEG_DEV || did == MAGSENS_DEV ) {
                Compass::createCompass(itf->getId());
            }
            else if ( did == FLARM_DEV ) {
                GpsVSensor::createGpsVSensor(); // create flarm processor
            }
        }
        else {
            // a sensor w/o a data link?
            if ( iid == OW_BUS && OneWIRE && did == TEMPSENS_DEV ) {
                dev->_sensor = OneWIRE->probeAndSetup(0x28); // DS18B20
            } else if ( iid == NO_PHY && did == TEMPSENS_DEV ) {
                dev->_sensor = new TempVSens();
            }
            
            if ( ! dev->_sensor ) {
                ESP_LOGW(FNAME, "Could not create sensor for device %d on itf %d", did, iid);
                delete dev;
                return nullptr;
            }
                
        }
    }

    EnumList pl;
    if ( dev->_link ) {
        // adding protocols does need the data link layer
        pl = dev->_link->addProtocol(proto, did, send_port); // Add proto, if not yet there
        dev->_protos.insert(pl.begin(), pl.end());
    }

    if ( is_new ) {
        std::lock_guard<SemaphoreMutex> lock(_devmap_mutex);
        // Add only if new
        _device_map[did] = dev; // and add, in case this dev is new
        _interface_map[itf->getId()] = itf;
    }
    refreshRouteCache();

    if ( nvsave ) {
        const DeviceAttributes &da = getDevAttr(did); // grab first hit with nvs entry
        if ( da.nvsetup && dev ) {
            // save it to nvs
            da.nvsetup->set(dev->getNvsData(), false, false);
        }
    }

    // ESP_LOGI(FNAME, "After add device %d.", did);
    // dumpMap();

    return dev;
}

Device *DeviceManager::getDevice(DeviceId did)
{
    DevMap::iterator it = _device_map.find(did);
    if ( it != _device_map.end() ) {
        return it->second;
    }
    return nullptr;
}

Device *DeviceManager::getXCVPeer()
{
    DevMap::iterator it = _device_map.find((xcv_role.get()==MASTER_ROLE)?XCVARIOSECOND_DEV:XCVARIOFIRST_DEV);
    if ( it != _device_map.end() ) {
        return it->second;
    }
    return nullptr;
}

ProtocolItf *DeviceManager::getProtocol(DeviceId did, ProtocolType proto)
{
    Device *d = getDevice(did);
    if ( d ) {
        return d->getProtocol(proto);
    }
    return nullptr;
}

NmeaPrtcl *DeviceManager::getNMEA(DeviceId did)
{
    Device *d = getDevice(did);
    if ( d && d->_link ) {
        return d->_link->getNmea();
    }
    return nullptr;
}

// convenience
int DeviceManager::getSendPort(DeviceId did, ProtocolType proto)
{
    Device *dev = getDevice(did);
    if ( dev ) {
        return dev->getSendPort(proto);
    }
    return -1;
}

// Remove device from map, delete device and all resources
// returns true, when a reboot is needed
bool DeviceManager::removeDevice(DeviceId did, bool nvsave)
{
    Device* dev = nullptr;
    {
        std::lock_guard<SemaphoreMutex> lock(_devmap_mutex);

        DevMap::iterator it = _device_map.find(did);
        if ( it != _device_map.end() ) {
            dev = it->second;
            ESP_LOGI(FNAME, "Delete device %d", did);
            _device_map.erase(it);
        }
    }
    bool ret = false;

    if ( dev ) {
        InterfaceCtrl *itf = dev->_itf;
        delete dev;
        // is it the last device on this interface
        if ( itf->getNrDLinks() == 0 ) {
            _interface_map.erase(itf->getId());
            if ( itf == CAN ) {
                ESP_LOGI(FNAME, "stopping CAN");
                CAN->stop();
            }
            else if ( itf == WIFI ) {
            // fixme, WiFi delete not reliably working (idf issue)
            //     ESP_LOGI(FNAME, "stopping Wifi");
            //     WIFI *tmp = WIFI;
            //     WIFI = nullptr;
            //     delete tmp;
                ret = true; // restart needed
            }
            else if ( itf == BLUEspp ) {
                ESP_LOGI(FNAME, "stopping BTspp");
                delete BLUEspp;
                BLUEspp = nullptr;
                ret = true; // restart needed
            }
            else if ( itf == BLUEnus ) {
                ESP_LOGI(FNAME, "stopping BTle");
                delete BLUEnus;
                BLUEnus = nullptr;
                ret = true; // restart needed
            }
            else if ( itf == S1 ) {
                ESP_LOGI(FNAME, "stopping S1");
                S1->stop();
            }
            else if ( itf == S2 ) {
                ESP_LOGI(FNAME, "stopping S2");
                S2->stop();
            }
            else if ( itf == OneWIRE ) {
                ESP_LOGI(FNAME, "stopping OneWire");
                delete OneWIRE;
                OneWIRE = nullptr;
            }
        }

        if ( nvsave ) {
            const DeviceAttributes &da = getDevAttr(did);
            ESP_LOGI(FNAME, "NV save removal for device %d", did);
            if ( da.nvsetup ) {
                // clear entry in nvs
                da.nvsetup->set(DeviceNVS(), false, false);
            }
        }
    }
    refreshRouteCache();

    ESP_LOGI(FNAME, "After remove device %d.", did);
    dumpMap();

    return ret;
}

// routing lookup table
InterfaceCtrl* DeviceManager::getIntf(DeviceId did)
{
    std::lock_guard<SemaphoreMutex> lock(_devmap_mutex);
    DevMap::iterator it = _device_map.find(did);
    InterfaceCtrl* tmp = nullptr;
    if ( it != _device_map.end() ) {
        tmp = it->second->_itf;
    }
    return tmp;
}

bool DeviceManager::isIntf(ItfTarget tgt) const
{
    auto it = _interface_map.find(tgt.iid);
    if ( it != _interface_map.end() ) {
        return true;
    }
    return false;
}

bool DeviceManager::isAvail(InterfaceId iid) const
{
    // CAN depends on HW revision only
    if ( iid == CAN_BUS && hardwareRevision.get() < XCVARIO_22 ) {
        return false;
    }
    // BT and WIFI are mutually exclusive
    else if ( (iid == BT_SPP || iid == BT_LE) && isIntf(WIFI_APSTA) ) {
        return false;
    }
    else if ( iid == WIFI_APSTA && (isIntf(BT_SPP) || isIntf(BT_LE)) ) {
        return false;
    }
    return true;
}


// Result is cashed for performance purpose.
RoutingList DeviceManager::getRouting(RoutingTarget source)
{
    // find routing entry, respect NO_PHY wildcards
    const RoutingTarget *route_list = findRoute(source); // all routes for the given source
    ESP_LOGD(FNAME, "Search route: %x(%d:%d:%d)", (unsigned)source.raw, source.getDeviceId(), source.getItfTarget().iid, source.getItfTarget().port);
    if ( route_list )
    {
        RoutingList activeList;

        // look for existing devices and push only those to the routing list
        for (const RoutingTarget* entry = route_list; entry->raw != 0; ++entry)
        {
            ESP_LOGD(FNAME, "Try match: d%d/i%d/o%d", entry->did, entry->getItfId(), entry->getPort());
            // find a configured dev as on the route
            DevMap::iterator devit = _device_map.find(entry->did);
            if ( devit != _device_map.end() ) {

                // check if itf and  port align
                Device *dev = devit->second;
                ESP_LOGD(FNAME, "Found dev %d on itf: %d", devit->first, dev->_itf->getId());
                if ( dev->_itf->getId() == entry->getItfId() ) {
                    ESP_LOGD(FNAME, "Itfmatch: %d==%d", dev->_itf->getId(), dev->_itf->getId());
                    EnumList pl = dev->getSendPortList();
                    if ( entry->getPort()==0 && pl.size()==1) {
                        // Take this joker port one
                        RoutingTarget merget = *entry;
                        merget.setItfPort(*pl.begin());
                        ESP_LOGD(FNAME, "Take joker port %d, d%d/i%d/p%d", *pl.begin(), merget.did, merget.getItfId(), merget.getPort());
                        activeList.insert(merget);
                    }
                    else if ( pl.find(entry->getItfTarget().port) != pl.end() ) {
                        ESP_LOGD(FNAME, "Take route d%d/i%d/p%d", entry->did, entry->getItfId(), entry->getPort());
                        activeList.insert(*entry); // Ok, take this route
                    }
                }
            }
        }
        return activeList; // will be a move by RVO
    }
    else {
        return RoutingList();
    }
}

// Refresh routes after a change of the devices map
void DeviceManager::refreshRouteCache()
{
    for ( auto dev : _device_map ) {
        // all devices
        if ( dev.second->_link )dev.second->_link->updateRoutes(dev.second->_id);
    }
}

void DeviceManager::setFlarmBPInitiator(DataLink *dl)
{
    _flarm_bp = dl;
}

// Start a binary data route
DataLink *DeviceManager::getFlarmBPInitiator()
{
    return _flarm_bp;
}

void DeviceManager::reserectFromNvs()
{
    // go through all possible setup entries and see if there is some thing valid
    int nr_set_up = 0;
    for (const auto& entry : DEVATTR) {
        // check nvsetup exists - otherwise a reserection is not desired (auto setup)
        // check if store data has the valid tag
        // check on role compatibility
        // skip extra "details" entries in the dev asttributes table
        ESP_LOGI(FNAME, "Read attr Entry: did%d >%s< t%x", entry.first, entry.second.name.data(),
                 entry.second.nvsetup ? (unsigned)entry.second.nvsetup->get().target.raw : 0 );
        if ( entry.second.nvsetup && *entry.second.name.data() != '\0' && entry.second.nvsetup->get().isValid()
            && ( !entry.second.getRoleDep() || entry.second.getRoleDep() == xcv_role.get()) ) {
            // setup device
            DeviceNVS *nvs = static_cast<DeviceNVS*>(entry.second.nvsetup->getPtr());
            ESP_LOGI(FNAME, "NVSEntry: t%x - s%x (%d/%d)", (unsigned)nvs->target.raw, (unsigned)nvs->setup.data, nvs->bin_sp, nvs->nmea_sp);
            DeviceId did = nvs->target.getDeviceId();
            int listen_port = nvs->target.getItfTarget().port;
            InterfaceId iid = nvs->target.getItfTarget().iid;
            ESP_LOGI(FNAME, "Reserect: did%d p%d iid%d", did, listen_port, iid);
            // bin proto option
            if ( nvs->setup.getProto(0) ) {
                ESP_LOGI(FNAME, "BinP: pid%d sp%d", nvs->setup.getProto(0), nvs->getBinSPort());
                addDevice(did, nvs->setup.getProto(0), listen_port, nvs->getBinSPort(), iid);
                nr_set_up++;
            }
            // principle nmea proto option
            ESP_LOGI(FNAME, "Nmea: sendport%d", nvs->getNmeaSPort());
            // more nmea plug options
            for ( int i=1; i<ProtocolList::maxProto; i++ ) {
                if ( nvs->setup.getProto(i) ) {
                    ESP_LOGI(FNAME, "plugin: %d", nvs->setup.getProto(i));
                    addDevice(did, nvs->setup.getProto(i), listen_port, nvs->getNmeaSPort(), iid);
                    nr_set_up++;
                }
            }
            // update my caps
            CANPeerCaps::updateCapsFromDev(did, true);
        }
    }
    ESP_LOGI(FNAME, "Reserected %d dev entries from NVS", nr_set_up);
}

void DeviceManager::introduceDevices()
{
    // A very first start w/ devices. Add a Flarm automatically
    assert(S1);
    S1->ConfigureIntf(SM_FLARM); // load flarm serial default profile
    Device *dev = DEVMAN->addDevice(FLARM_DEV, FLARM_P, 0, 0, S1_RS232);
    DEVMAN->addDevice(FLARM_DEV, FLARMBIN_P, 0, 0, NO_PHY);
    if ( dev ) {
        // save it to nvs
        flarm_devsetup.set(dev->getNvsData());
    }
    if ( S2 ) {
        S2->ConfigureIntf(SM_XCTNAV_S3); // load XCTouchNav serial default profile
        dev = DEVMAN->addDevice(NAVI_DEV, XCVARIO_P, 0, 0, S2_RS232);
        if ( dev ) { navi_devsetup.set(dev->getNvsData()); }
        dev = DEVMAN->addDevice(FLARM_HOST_DEV, FLARMHOST_P, 0, 0, S2_RS232);
        if ( dev ) { flarm_host_setup.set(dev->getNvsData()); }
    }
    if ( CAN && (can_speed.get()!=CAN_SPEED_1MBIT) ) {
        // Set it once to 1Mbit
        can_speed.set(CAN_SPEED_1MBIT);
        CAN->ConfigureIntf(1);
    }
}

// prio - 0,..,4 0:low prio data stream, .. 5: important high prio commanding
// return an id chunk of four to use (0..0x3fc), use then +0, +1, +2, +3
// else -1 in case there is no chunk left
static int nxt_free_slot[5] = { 0x400, 0x200, 0x100, 0x80, 0x40 }; // 5 prio slots of 15 chucks of 4 ids
int DeviceManager::reserveCANId(int prio)
{
    if ( prio > 4 ) {
        prio = 4;
    }
    if ( prio < 0 ) {
        prio = 0;
    }
    if ( (nxt_free_slot[prio] & 0x3c) != 0x3c ) { // chunk mask b0011 1100
        int slot = nxt_free_slot[prio];
        nxt_free_slot[prio] += 4; // next chunk of four
        return slot;
    }
    return -1;
}

void DeviceManager::undoReserveCANId(int prio)
{
    if ( prio > 4 ) {
        prio = 4;
    }
    if ( prio < 0 ) {
        prio = 0;
    }
    if ( (nxt_free_slot[prio] & 0x3c) >= 4 ) {
        nxt_free_slot[prio] -= 4; // revert reservation
    }
}

void DeviceManager::dumpMap() const
{
    ESP_LOGI(FNAME, "Device map dump:");
    for ( auto &it : _device_map ) {
        Device* d = it.second;
        ESP_LOGI(FNAME, "%d: %p (did%d/%s(%d))", it.first, d, d->_id, d->_itf->getStringId(), d->_itf->getId());
        if ( d->_link ) d->_link->dumpProto();
    }
}

void DeviceManager::startMonitoring(ItfTarget tgt)
{
    // all interfaces
    for ( auto itf : _interface_map ) {
        itf.second->startMonitoring(tgt);
    }
    // activate also the send callback
    monitor_target = tgt;
}

void DeviceManager::stopMonitoring()
{
    // stop send monitor
    monitor_target = ItfTarget();
    // all interfaces
    for ( auto itf : _interface_map ) {
        itf.second->stopMonitoring();
    }
}

std::vector<const Device*> DeviceManager::allDevs() const
{
    std::vector<const Device*> ret;
    for ( auto &it : _device_map ) {
        ret.push_back(it.second);
    }
    return ret;
}

void DeviceManager::EnforceIntfConfig(InterfaceId iid, DeviceId did)
{
    if ( iid == S1_RS232 || iid == S2_RS232 ) {
        InterfaceCtrl *itf = (iid == S1_RS232) ? S1 : ((iid == S2_RS232) ? S2 : nullptr);
        if ( ! itf ) { return; } // no interface, nothing to do
        switch (did) {
        case FLARM_DEV:
            itf->ConfigureIntf(SM_FLARM); // load flarm serial default profile
            break;
        case NAVI_DEV: // todo respect flavors
        case FLARM_HOST_DEV:
        case RADIO_REMOTE_DEV:
            itf->ConfigureIntf(SM_XCTNAV_S3); // load XCTouchNav serial default profile
            break;
        case RADIO_KRT2_DEV:
        case RADIO_ATR833_DEV:
            itf->ConfigureIntf(SM_RADIO); // load radio serial default profile
            break;
        default:
            break;
        }
    }
    else if ( iid == CAN_BUS ) {
        // CAN bus, set 1000kbit
        can_speed.set(CAN_SPEED_1MBIT); // 1000 kbit/s
        if ( CAN ) {
            CAN->ConfigureIntf(1);
        }
    }
    // else if ( iid == OW_BUS ) {
    // }
    else {
        ESP_LOGW(FNAME, "no interface config needed %d", iid);
    }
}


//
// A Device
//

// Resolve the existance of a device
Device::~Device()
{
    ESP_LOGI(FNAME, "Dtor device %d.", _id);
    if ( _link )
    {
        // Detach data links from interface
        if ( _link->decrDeviceCount() == 0 ) {
            // last device on this link
            ESP_LOGI(FNAME, "Last device on %s port %d", _itf->getStringId(), _link->getPort());
            _itf->DeleteDataLink(_link->getPort());
        }
        else {
            ESP_LOGI(FNAME, "Still %d devices on %s, remove protos.", _link->getDeviceCount(), _itf->getStringId());
            for (auto it : _protos) {
                _link->removeProtocol(static_cast<ProtocolType>(it));
            }
            // _link->removeId(_id); not needed
        }
    }
    if ( _sensor ) {
        ESP_LOGI(FNAME, "Delete sensor for device %d.", _id);
        delete _sensor;
        // _sensor = nullptr;
    }
}

ProtocolItf *Device::getProtocol(ProtocolType p) const
{
    // Find protocol
    if ( _link ) {
        ProtocolItf *tmp = _link->getProtocol(p);
        if ( tmp ) {
            return tmp;
        }
    }
    return nullptr;
}

DataLink *Device::getDLforProtocol(ProtocolType p) const
{
    // Find protocol
    if ( _link ) {
        if ( _link->getProtocol(p) ) {
            return _link;
        }
    }
    return nullptr;
}

EnumList Device::getSendPortList() const
{
    EnumList pl;
    if ( _link ) {
        for ( int p : _link->getAllSendPorts()) {
            pl.insert(p);
        }
    }
    return pl;
}

int Device::getListenPort() const
{
    return (_link) ? _link->getPort() : 0;
}

// get all setup lines to write into nvs for this device
DeviceNVS Device::getNvsData() const
{
    ESP_LOGI(FNAME, "NvsData did%d", _id);
    DeviceNVS entry;
    if ( ! _link ) {
        entry.setup.flags=0; // fixme
        return entry;
    }
    entry.target = RoutingTarget(_id, _link->getTarget());
    ProtocolItf *bn = _link->getBinary(); // binary option
    if ( bn ) {
        entry.bin_sp = bn->getSendPort();
        entry.setup.set(0, bn->getProtocolId());
        ESP_LOGI(FNAME, "NvsData t%x bsp%d pid%d", (unsigned)entry.target.raw, entry.bin_sp, entry.setup.get(0));
    }
    NmeaPrtcl *nm = static_cast<NmeaPrtcl*>(_link->getNmea());

    if ( nm ) {
        entry.nmea_sp = nm->getSendPort();
        // entry.setup.set(1, nm->getProtocolId());
        int idx = 1;
        ESP_LOGI(FNAME, "NvsData t%x nmsp%d", (unsigned)entry.target.raw, entry.nmea_sp);
        for (auto it : nm->getAllPlugs() ) {
            // list when not auto loaded, but does belong to this device
            if ( ! it->getAuto() && it->getRouteId() == _id ) {
                entry.setup.set(idx, it->getPtyp()); // plugins
                ESP_LOGI(FNAME, "NvsExtra %d: pid%d", idx, entry.setup.get(idx));
                idx++;
            }
            else {
                ESP_LOGI(FNAME, "NvsDropped  pid%d did%d", it->getPtyp(), _id);
            }
            if ( idx >= ProtocolList::maxProto ) {
                ESP_LOGW(FNAME, "Nvs no space left %d: pid%d", idx, entry.setup.get(idx));
                break; // no more space
            }
        }
    }
    ESP_LOGI(FNAME, "Nvs valid did%d t%x s%x bsp%d nsp%d", _id, (unsigned)entry.target.raw, (unsigned)entry.setup.data, entry.bin_sp, entry.nmea_sp);
    entry.setup.flags=1; // set valid
    return entry;
}


int Device::getSendPort(ProtocolType p) const
{
    // get port for protocol
    ProtocolItf *tmp = getProtocol(p);
    if ( tmp ) {
        return tmp->getSendPort();
    }

    return -1; // invalid port
}



// A set of convenience routines to use the pool
namespace DEV
{

Message* plsMessage(DeviceId target_id, int port)
{
    Message* m = MP.getOne(false);
    if ( m ) {
        m->target_id = target_id;
        m->port = port;
        return m;
    }
    return nullptr;
}

Message* acqMessage(DeviceId target_id, int port)
{
    Message* m = MP.getOne();
    m->target_id = target_id;
    m->port = port;
    return m;
}

void relMessage(Message *msg)
{
    MP.recycleMsg(msg);
}

bool Send(Message* msg)
{
    if ( pdTRUE != xQueueSend( ItfSendQueue, (void * ) &msg, portMAX_DELAY ) ) {
        // drop it
        ESP_LOGW(FNAME, "Dropped message to %d", msg->target_id);
        MP.recycleMsg(msg);
        return false;
    }
    return true;
}

} // namespace
