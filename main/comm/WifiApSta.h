#pragma once

#include "comm/InterfaceCtrl.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <lwip/sockets.h>


#define NUM_TCP_PORTS 5
#define MAX_CLIENTS   4

// extern bool netif_initialized;
extern const char* AP_PASSPHARSE;
extern const char* SSID_PREFIX;
extern const char* OTA_SSID;

class WifiApSta;
extern WifiApSta *WIFI; // global wifi access

struct peer_record {
	int peer = -1;
	int retries = 0;
	struct sockaddr_in clientAddress;
};

enum SocketState : uint8_t {
    DOWN,
    WAIT_IP,    // only for station mode
    HAVE_IP,
    CONNECTED,
    LISTENING   // only for AP mode
};

struct sock_ctrl_t {
	sock_ctrl_t(int p);;
	int create_ap_socket();
	int connect_sta_socket();
    int nextFreePeer();
    constexpr void operator--(int) { if (_nr_peers > 0) --_nr_peers; }
	// This should store the handle to send data of the port
	const int port;
	int sock_hndl = -1;
	peer_record peers[MAX_CLIENTS];
    uint8_t _nr_peers = 0; 
	uint8_t is_ap : 1 = true;
    uint8_t sta_auth_ok : 1 = true; // set to false on a authentication failure
	uint8_t alive : 1 = false;
    uint8_t state : 6 = SocketState::DOWN;
};

class WifiApSta final : public InterfaceCtrl
{
	friend class WIFI_EVENT_HANDLER;

private:
	WifiApSta();
public:
	static WifiApSta *createWifiApSta();
	~WifiApSta();

public:
	// Ctrl
	InterfaceId getId() const override { return WIFI_APSTA; }
	const char *getStringId() const override { return "WiFi"; }
	void ConfigureIntf(int port) override; // 8880, 8881, 8882, 8883, 8884, 80
	virtual int Send(const char *msg, int &len, int port = 0) override;

	bool isAlive(); // returns true if AP is up and running
	bool isAP() const { return _ap_netif != nullptr; }
	bool isSTA() const { return _sta_netif != nullptr; }
	// bool isConnected(int p) const;
	bool scanMaster(int master_xcv_num);

private:
    uint8_t socket_state = 0;
	sock_ctrl_t *_socks[NUM_TCP_PORTS];
	TaskHandle_t socket_server_task_pid = nullptr;
	bool _terminte_sock_server = false;
	esp_netif_t *_ap_netif = nullptr;
	esp_netif_t *_sta_netif = nullptr;
	esp_event_handler_instance_t _wifi_evnt_handler = nullptr;
	esp_event_handler_instance_t _ip_evnt_handler = nullptr;

	// internal functionality
	bool initialize_wifi(bool ap_mode, int maxcon, const char* ssid);
	void client_reconnect();
};

