
#include "WifiApSta.h"
#include "setup/SetupNG.h"
#include "setup/SetupCommon.h"
#include "comm/DataLink.h"
#include "driver/gpio/ESPRotary.h"
#include "logdef.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <lwip/sockets.h>
#include <esp_system.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_event.h>
#include <mutex>

bool netif_initialized = false;

WifiApSta *WIFI = nullptr;


// The XCV access point password
const char* AP_PASSPHARSE = "xcvario-21";
// The XCV access point SSID prefix
const char* SSID_PREFIX = "XCVario-";
// The XCV software update access point SSID
const char* OTA_SSID = "ESP32 OTA";

enum WifiEventType : uint8_t {
    NO_EVENT,
    STA_STARTED,
    STA_CONNECTED,
    STA_GOT_IP,
    STA_DISCONNECTED,
    AP_STARTED,
    TERMINATE
};

struct WifiEvent {
    WifiEventType type;
    uint32_t      data;
    constexpr WifiEvent() : type(NO_EVENT), data(0) {}
    constexpr WifiEvent(WifiEventType t, uint32_t d = 0) : type(t), data(d) {}
};

QueueHandle_t wifi_queue = nullptr;

union NetState {
    enum StaState : uint8_t {
    WIFI_OFF,
    WIFI_STARTED,
    WIFI_CONNECTED,
    WIFI_GOT_IP,
    WIFI_STA_UP
    };
    struct {
        StaState wifi   : 4;
        uint8_t  nrsock : 4; // nr of open sockets
    };
    constexpr NetState() : wifi(WIFI_OFF), nrsock(0) {}
    constexpr bool isSockOpen() const { return nrsock > 0; }
    constexpr bool isStaUp() const { return wifi == WIFI_STA_UP; }
    constexpr bool isStaGotIP() const { return wifi == WIFI_GOT_IP; }
    constexpr void reset() { wifi = WIFI_OFF; nrsock = 0; }
    constexpr void operator++(int) { ++nrsock; }
    constexpr void operator--(int) { if (nrsock > 0) --nrsock; }
    constexpr void setStaState(StaState s) { wifi = s; }
};

sock_ctrl_t::sock_ctrl_t(int p) : port(p) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        peers[i] = peer_record();
    }
}

int sock_ctrl_t::create_ap_socket()
{
	struct sockaddr_in serverAddress;
	// Create a socket that we will listen upon.
	int mysock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (mysock < 0) {
		ESP_LOGE(FNAME, "socket: %d %s", mysock, strerror(errno));
		return -1;
	}
	int opt = 1;
	setsockopt(mysock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	// Bind our server socket to a port.
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(port);

	int rc  = bind(mysock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	if (rc < 0) {
		ESP_LOGE(FNAME, "bind: %d %s", rc, strerror(errno));
		return -1;
	}
	ESP_LOGV(FNAME, "bind port: %d", port  );

	// Flag the socket as listening for new connections.
	rc = listen(mysock, 5);
	if (rc < 0) {
		ESP_LOGE(FNAME, "listen: %d %s", rc, strerror(errno));
		return -1;
	}
	int flag = 1; // prepare for short telegram transmission
	setsockopt(mysock, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

	sock_hndl = mysock;
	ESP_LOGI(FNAME, "socket %d listen successfully!", mysock);

	return mysock;
}

int sock_ctrl_t::connect_sta_socket()
{
	// Connect to XCVario master
	struct sockaddr_in dest_addr;
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(port);
	dest_addr.sin_addr = peers[0].clientAddress.sin_addr; // ip_info->gw.addr;
	ESP_LOGI(FNAME, "connect STA to ip: %x", (unsigned)dest_addr.sin_addr.s_addr);

	int mysock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // IPPROTO_IP);
	if (mysock < 0) {
		ESP_LOGE(FNAME, "failed to allocate socket: %s, port %d", strerror(errno), port );
		sock_hndl = -1;
		return -1;
	}
	if (connect(mysock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
		ESP_LOGE(FNAME, "socket connect failed: %s, port %d", strerror(errno ), port );
		close(mysock);
		return -1;
	}
	vTaskDelay(pdMS_TO_TICKS(5000)); // that seems to avout the select errno 22
	int flag = 1; // prepare for short telegram transmission
	setsockopt(mysock, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

	sock_hndl = mysock;

	ESP_LOGI(FNAME, "socket %d connected successfully!", mysock);
	return mysock;
}

int sock_ctrl_t::nextFreePeer()
{
    if ( _nr_peers < MAX_CLIENTS ) {
        _nr_peers++;
        // return the next free index
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (peers[i].peer == -1) {
                return i;
            }
        }
    }
    return -1;
}

class WIFI_EVENT_HANDLER
{
public:

	// WiFi Task
	static void socket_server(void *arg)
	{
		WifiApSta *wifi = static_cast<WifiApSta *>(arg);
		int nosock_counter = 0;
        wifi_queue = xQueueCreate(10, sizeof(WifiEvent));
	    NetState state = NetState();
        WifiEvent evt;

		while (true) {

            // wait for wifi events, but only if no sockets are open, otherwise directly go to select
            TickType_t wait_time = (state.isSockOpen() || state.isStaUp()) ? 0 : portMAX_DELAY;
            while (xQueueReceive(wifi_queue, &evt, wait_time) == pdTRUE)
            {
                switch (evt.type)
                {
                case WifiEventType::STA_STARTED: {
                    state.setStaState(NetState::WIFI_STARTED);
                    
                    sock_ctrl_t* xcvcl = wifi->_socks[4];
                    ESP_LOGI(FNAME, "SYSTEM_EVENT_STA_START hndl %d ap %d", xcvcl ? xcvcl->sock_hndl : -2, xcvcl ? xcvcl->is_ap : -2);
                    if (xcvcl && xcvcl->sock_hndl < 0 && !xcvcl->is_ap) {
                        // vTaskDelay(pdMS_TO_TICKS(1000));
                        // mywifi->client_connect();
                        ESP_ERROR_CHECK(esp_wifi_connect());
                    }
                    esp_wifi_connect();
                    break;
                }
                case WifiEventType::STA_CONNECTED: {
                    state.setStaState(NetState::WIFI_CONNECTED);

                    ESP_LOGI(FNAME, "STA disconnected, reason=%d", (int)evt.data);
                    sock_ctrl_t* xcvcl = wifi->_socks[4];
                    if (evt.data == WIFI_REASON_AUTH_FAIL) {
                        // fatal, do not try to reconnect, just mark the connection as failed
                        if (xcvcl) {
                            xcvcl->sta_auth_ok = false;
                        }
                        break;
                    }
                
                    if (xcvcl && !xcvcl->is_ap) {
                        if (xcvcl->sock_hndl > 0) {
                            close(xcvcl->sock_hndl);
                            xcvcl->sock_hndl = -1;
                            xcvcl->alive = false;  // mark as dead
                            xcvcl->_nr_peers = 0;
                        }
                        ESP_ERROR_CHECK(esp_wifi_connect());
                    }

                    break;
                }
                case WifiEventType::STA_GOT_IP: {
                    state.setStaState(NetState::WIFI_GOT_IP);
                    esp_ip4_addr* evt_ip = (esp_ip4_addr*)&evt.data;
                    ESP_LOGI(FNAME, "SYSTEM_EVENT_STA_GOT_IP gateway:" IPSTR, IP2STR(evt_ip));
                    sock_ctrl_t* xcvcl = wifi->_socks[4];
                    if (xcvcl) {
                        if (xcvcl->sock_hndl >= 0) {
                            ESP_LOGI(FNAME, "need to recreate the sta  socket");
                            close(xcvcl->sock_hndl);
                            xcvcl->alive = false;  // mark as dead
                        }
                        peer_record new_rec;
                        new_rec.clientAddress.sin_addr.s_addr = evt_ip->addr;
                        xcvcl->_nr_peers = 0;
                        xcvcl->peers[0] = new_rec;  // sta has just one peer, eg the master XCVario
                        xcvcl->_nr_peers = 1;
                        xcvcl->sock_hndl = -1;              // this is the indicator to connect to the AP
                        xcvcl->is_ap = false;
                        xcvcl->state = SocketState::HAVE_IP;
                    }

                        // only from here on sockets are allowed
                        // WIFI_EVENT_HANDLER::socket_manager_start();
                        break;
                }
                case WifiEventType::STA_DISCONNECTED: {
                    state.setStaState(NetState::WIFI_STARTED);

                    // WIFI_EVENT_HANDLER::socket_manager_stop();   // safe cleanup only
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_wifi_connect();
                    break;
                }
                case WifiEventType::AP_STARTED: {
                    ESP_LOGI(FNAME, "AP started");
                    state++; // new AP socket opened
                    break;
                }
                case WifiEventType::TERMINATE: {
                    vTaskDelete(NULL);
                    // state.setStaState(NetState::WIFI_OFF);

                    // WIFI_EVENT_HANDLER::socket_manager_stop();   // safe cleanup only
                    // vTaskDelay(pdMS_TO_TICKS(1000));
                    // esp_wifi_disconnect();
                    break;
                }
                default:
                    break;
                }
                wait_time = (state.isSockOpen()) ? 0 : portMAX_DELAY; // reconsider wait time
            }


            /////////////////////////////////////////////////////////////////////
            // prepare the fd_set for select
			sock_ctrl_t *config;
			int max_fd = 0;
			fd_set read_fds;
			FD_ZERO(&read_fds);

			// prepare select condition
			for (int n = 0; n < NUM_TCP_PORTS; n++) {
				if (wifi->_socks[n] == nullptr) {
					continue;
				}

				config = wifi->_socks[n];
				if ( config->sock_hndl >= 0 ) {
					if ( config->sock_hndl == 0 ) {
						if ( config->connect_sta_socket() < 0 ) continue;
					}
					ESP_LOGI(FNAME, "sock server, port: %d, clients: %d", 8880+n, config->_nr_peers );

					FD_SET(config->sock_hndl, &read_fds);
					max_fd = std::max(max_fd, config->sock_hndl);
				}
				for (int i = 0; i < MAX_CLIENTS; i++) {
					peer_record &client_rec = config->peers[i];
					if (client_rec.peer >= 0) {
						FD_SET(client_rec.peer, &read_fds);
						max_fd = std::max(max_fd, client_rec.peer);
					}
				}
			}
			if ( max_fd == 0 ) {
				ESP_LOGW(FNAME, "No sockets to monitor, sleeping for 1s");
				vTaskDelay(pdMS_TO_TICKS(1000)); // No sockets to monitor, sleep for a while
				nosock_counter++;
				// if ( nosock_counter > 10 ) {
				// 	if ( wifi->_sta_netif != nullptr ) {
				// 		ESP_LOGW(FNAME, "No sockets available for 10 seconds, try reconnecting");
				// 		wifi->client_reconnect();
				// 	}
				// 	nosock_counter = 0;
				// }
				continue;
			}
			nosock_counter = 0;

			//////////////////////////////////////////////////////////////////////
			// block max 1sec to allow controlling of this task, incl. termination
			struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };;
			int select_result = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);
			if ( wifi->_terminte_sock_server ) {
				ESP_LOGI(FNAME, "socket_server: received terminate signal");
				break;
			}
			if (select_result <= 0) {
				// Timeout occurred, or an error occurred
				if ( select_result < 0 && errno != EINTR ) {
					vTaskDelay(pdMS_TO_TICKS(1000));
					ESP_LOGW(FNAME, "select result: %d (errno %d)", select_result, errno);
				}
				continue;
			}

			for (int n = 0; n < NUM_TCP_PORTS; n++) {
				if (wifi->_socks[n] == nullptr) {
					continue;
				}
				config = wifi->_socks[n];
				bool tmp_alive = false;

				if ( FD_ISSET(config->sock_hndl, &read_fds) ) {
					ESP_LOGI(FNAME, "FD_ISSET socket: %d num clients %d, port %d", config->sock_hndl, config->_nr_peers, config->port );
					if ( config->is_ap ) {
						// handle incoming connection on the server socket
						if ( config->_nr_peers < MAX_CLIENTS ) {
							struct sockaddr_in clientAddress;
							socklen_t clientAddressLength = sizeof(clientAddress);
							int new_client = accept(config->sock_hndl, (struct sockaddr *)&clientAddress, &clientAddressLength);
							if (new_client >= 0) {
                                int free_peer_idx = config->nextFreePeer();
                                if (free_peer_idx < 0) {
                                    ESP_LOGW(FNAME, "No free peer slot available for new client on port %d, rejecting connection", config->port);
                                    close(new_client);
                                    continue;
                                }
								peer_record &new_rec = config->peers[free_peer_idx];
								new_rec.peer = new_client;
								new_rec.retries = 0;
								new_rec.clientAddress = clientAddress;
								ESP_LOGI(FNAME, "New client: %d, number of clients: %d", new_client, config->_nr_peers);
							}
						}
						else {
							ESP_LOGW(FNAME, "Max clients reached for port %d, cannot accept new client", config->port);
						}
					} else {
						// This is a client socket, we can read from it
						ESP_LOGI(FNAME, "FD_ISSET client socket: %d, port %d", config->sock_hndl, config->port );
						char r[ProtocolItf::MAX_LEN + 1];
						ssize_t sizeRead = recv(config->sock_hndl, r, ProtocolItf::MAX_LEN - 1, MSG_DONTWAIT);
						if (sizeRead > 0) {
							ESP_LOGI(FNAME, "FD socket recv: connection %d, port %d, read:%d bytes", config->sock_hndl, config->port, sizeRead );
							tmp_alive = true;
							DataLink* dltarget = nullptr;
							{
								std::lock_guard<SemaphoreMutex> lock(wifi->_dlink_mutex);
								auto dl = wifi->_dlink.find(config->port);
								if ( dl != wifi->_dlink.end() ) {
									dltarget = dl->second;
								}
							}
							if (dltarget) {
								dltarget->process(r, sizeRead);
							}
						}
                        else if (sizeRead <= 0) {
							// Server closed the connection, or error occurred
							ESP_LOGI(FNAME, "Client %d error", config->sock_hndl);
							shutdown(config->sock_hndl, SHUT_RDWR);
							close(config->sock_hndl);
							config->alive = false; // mark as dead
							config->sock_hndl = -1; // mark as disconnected
                            WifiEvent disconnect_evt(WifiEventType::STA_DISCONNECTED);
                            xQueueSend(wifi_queue, &disconnect_evt, 0);
						}
					}
				}
				if ( n == 4 ) {
					continue; // skip the STA socket
				}

				for (int i = 0; i < MAX_CLIENTS; i++) {
					peer_record &client_rec = config->peers[i];
					if (client_rec.peer >=0 && FD_ISSET(client_rec.peer, &read_fds)) {
                        
						ESP_LOGI(FNAME, "FD_ISSET socket: connection %d, port %d", client_rec.peer, config->port );
						char r[ProtocolItf::MAX_LEN + 1];
						ssize_t sizeRead = recv(client_rec.peer, r, ProtocolItf::MAX_LEN - 1, MSG_DONTWAIT);
						if (sizeRead > 0) {
							ESP_LOGI(FNAME, "FD socket recv: connection %d, port %d, read:%d bytes", client_rec.peer, config->port, sizeRead );
							tmp_alive = true;
							DataLink* dltarget = nullptr;
							{
								std::lock_guard<SemaphoreMutex> lock(wifi->_dlink_mutex);
								auto dl = wifi->_dlink.find(config->port);
								if ( dl != wifi->_dlink.end() ) {
									dltarget = dl->second;
								}
							}
							if (dltarget) {
								dltarget->process(r, sizeRead);
							}
							client_rec.retries = 0;
						}
						else if (sizeRead <= 0) {
							// Client closed the connection, or error occurred
							ESP_LOGI(FNAME, "Client %d disconnected", client_rec.peer);
							shutdown(client_rec.peer, SHUT_RDWR);
							close(client_rec.peer);
							client_rec.peer = -1; // mark as disconnected
							config->_nr_peers--; // decrement the number of peers
							continue; // Skip to the next iteration
						}
					}
					else {
						client_rec.retries++;
					}
					if (client_rec.retries > 100) {
						ESP_LOGW(FNAME, "tcp client %d (port %d) permanent send error: %s, removing!", client_rec.peer, config->port, strerror(errno));
						close(client_rec.peer);
                        client_rec.peer = -1; // mark as disconnected
                        config->_nr_peers--; // decrement the number of peers
						continue; // Skip to the next iteration
					}
				}
				config->alive = tmp_alive; // mark as alive if we got some data

				if (uxTaskGetStackHighWaterMark(wifi->socket_server_task_pid) < 128) {
					ESP_LOGW(FNAME, "Warning wifi task stack low: %d bytes, port %d", uxTaskGetStackHighWaterMark(wifi->socket_server_task_pid), config->port);
				}
			}
		}
		vTaskDelete(NULL);
		wifi->socket_server_task_pid = nullptr;
	}

    static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        WifiEvent evt;
        if (event_base == WIFI_EVENT) {
            switch (event_id) {
            case WIFI_EVENT_STA_START: {
                evt.type = WifiEventType::STA_STARTED;
                ESP_LOGI(FNAME, "SYSTEM_EVENT_STA_START");
                break;
            }
            case WIFI_EVENT_STA_CONNECTED: {
                evt.type = WifiEventType::STA_CONNECTED;
                ESP_LOGI(FNAME, "STA connected to AP");
                break;
            }
            case WIFI_EVENT_STA_DISCONNECTED: {
                evt.type = WifiEventType::STA_DISCONNECTED;
                evt.data = ((wifi_event_sta_disconnected_t*)event_data)->reason;
                break;
            }
            case WIFI_EVENT_AP_STACONNECTED: {
                [[maybe_unused]] wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
                ESP_LOGI(FNAME, "station %x:%x:%x:%x:%x:%x joined, AID=%d", MAC2STR(event->mac), event->aid);
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED: {
                [[maybe_unused]] wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
                ESP_LOGI(FNAME, "station %x:%x:%x:%x:%x:%x left, AID=%d", MAC2STR(event->mac), event->aid);
                break;
            }
            default:
                break;
            }
        } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            evt.type = WifiEventType::STA_GOT_IP;
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(FNAME, "SYSTEM_EVENT_STA_GOT_IP ip:" IPSTR, IP2STR(&event->ip_info.ip));
            evt.data = event->ip_info.gw.addr; // pass the gateway address as data, that is the XCVario master IP in station mode
        }
        if (wifi_queue && evt.type != NO_EVENT  ) { // only send if event is set, otherwise we might block the queue with empty events
            xQueueSend(wifi_queue, &evt, 0);
        }
    }

}; // WIFI_EVENT_HANDLER


WifiApSta::WifiApSta() :
	InterfaceCtrl()
{
	// todo create sockets as needed and for other ports
	for(int i=0; i<NUM_TCP_PORTS; i++) {
		_socks[i] = nullptr;
	}

    // create the one and only task for STA and AP and socket management
    xTaskCreate(&WIFI_EVENT_HANDLER::socket_server, "wifitask", 4096, this, 8, &socket_server_task_pid);
    ESP_LOGI(FNAME, "SocketServerTask started: %p", socket_server_task_pid );
}

WifiApSta *WifiApSta::createWifiApSta()
{
	if ( ! WIFI ) {
		WIFI = new WifiApSta();
	}
	return WIFI;
}

WifiApSta::~WifiApSta()
{
    // fixme still needs work to reliably restart WiFi, 
    // currently no crash, but a disfunctional WiFi after restart.
    
	// stop the server task
	_terminte_sock_server = true;
    int16_t wait_counter = 22;
    do {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_counter--;
    } while (socket_server_task_pid != nullptr && wait_counter > 0); // wait for the server task to terminate

	for (int i=0; i < NUM_TCP_PORTS; i++) {
		if (_socks[i]) {
			for (int c=0; c < MAX_CLIENTS; c++) {
				peer_record &client_rec = _socks[i]->peers[c];
				if (client_rec.peer >= 0) {
					shutdown(client_rec.peer, SHUT_RDWR);
					close(client_rec.peer);
				}
			}

			// Close the server socket
			shutdown(_socks[i]->sock_hndl, SHUT_RDWR);
			vTaskDelay(pdMS_TO_TICKS(100)); // let the server do the clean up work
			close(_socks[i]->sock_hndl);
			_socks[i] = nullptr;
		}
	}

    if (_wifi_evnt_handler) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, _wifi_evnt_handler);
        _wifi_evnt_handler = nullptr;
    }
    if (_ip_evnt_handler) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, _ip_evnt_handler);
        _ip_evnt_handler = nullptr;
    }
    if (_ap_netif) {
        ESP_LOGV(FNAME, "now esp_netif_destroy_default_wifi ap");
        esp_netif_destroy_default_wifi(_ap_netif);
        _ap_netif = nullptr;
    } else if (_sta_netif) {
        ESP_LOGV(FNAME,"now esp_netif_destroy_default_wifi sta");
        esp_netif_destroy_default_wifi(_sta_netif);
        _sta_netif = nullptr;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
}

bool  WifiApSta::isAlive(){
	int ret = 0;
	for ( int i=0; i<NUM_TCP_PORTS; i++ ) {
		if( _socks[i] && _socks[i]->alive ) {
			// ESP_LOGI(FNAME, "WQF: %d is full", 8880+i );
			ret = true;
			break;
		}
	}
	
	return ret;
}

// bool WifiApSta::isConnected(int port) const
// {
// 	if (port >= 8880 && port <= 8884) {
// 		int socket_num = port - 8880;
// 		if (_socks[socket_num]) {
// 			return _socks[socket_num]->alive;
// 		}
// 	}
// 	return false;
// }

bool WifiApSta::scanMaster(int master_xcv_num)
{
	ESP_LOGI(FNAME,"wifi scan");

	const int DEFAULT_SCAN_LIST_SIZE = 20;
	uint16_t number = DEFAULT_SCAN_LIST_SIZE;
	wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
	uint16_t ap_count = 0;
	memset(ap_info, 0, sizeof(ap_info));

	// search for this AP
	char mxcv[14];
    strcpy(mxcv, SSID_PREFIX);
	if( master_xcv_num != 0 ) {
		sprintf( mxcv+strlen(mxcv),"%d", master_xcv_num );
	}

	bool found = false;
	while( !found ) {
		ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
		ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
		ESP_LOGI(FNAME, "Total APs scanned = %u", ap_count);
		if( Rotary->readSwitch() ) {
			break;
		}
		ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
		for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
			ESP_LOGI(FNAME, "SSID \t%s", ap_info[i].ssid);
			ESP_LOGI(FNAME, "LEVEL: \t%d SSID: \t%s", ap_info[i].rssi, ap_info[i].ssid);
			ESP_LOGI(FNAME, "Hunt for %s", mxcv );
			if( strncmp( (char*)ap_info[i].ssid, mxcv, strlen(mxcv) ) == 0 ) {
				found = true;
				int master_xcvario_scanned;
				sscanf((char*)ap_info[i].ssid, "XCVario-%d", &master_xcvario_scanned);
				master_xcvario.set(master_xcvario_scanned);
				ESP_LOGI(FNAME, "SCAN found master XCVario: %s", (char*)ap_info[i].ssid);
				break;
			}
		}
	}
	return found;
}

///////////////////////////
// the Interface Config API
//
// it's mostly static IP a cfg depending of port
void WifiApSta::ConfigureIntf(int port) {
    bool isAP = !((xcv_role.get() == SECOND_ROLE) && (port == 8884));  // client -- master connection is the only STA connection
    if (port >= 8880 && port <= 8884) {
        if (port == 8884 && _sta_netif && _socks[8884 - 8880] && _socks[8884 - 8880]->sock_hndl <= 0) {
            ESP_LOGI(FNAME, "STA already configured, try to reconnect");
            // client_reconnect();
            ESP_ERROR_CHECK(esp_wifi_connect());
            return;  // STA already configured
        }
        char ssid_buf[20];
        ssid_buf[0] = '\0';
        if ((int)master_xcvario.get() != 0) {
            sprintf(ssid_buf, "%s%d", SSID_PREFIX, (int)master_xcvario.get());
        }
        initialize_wifi(isAP, MAX_CLIENTS, isAP ? SetupCommon::getID() : ssid_buf);
    } else if (port == 80) {
        initialize_wifi(true, 2, OTA_SSID);
        return;  // no socket server needed for the OTA webserver
    } else {
        ESP_LOGE(FNAME, "Invalid cfg: %d, should be between 8880 and 8884, or 80", port);
        return;
    }

    int pidx = port - 8880;
    sock_ctrl_t* sock = _socks[pidx];                 // particular pointer to socket server record for this port
    if (sock == nullptr) {                            // its a one-way train, we can only create those ATM
        _socks[pidx] = sock = new sock_ctrl_t(port);  // WiFiAP only once, all point to here
        sock->is_ap = isAP;
        if (isAP) {
            int sock_err = sock->create_ap_socket();  // Create already the socket for this port structure as interface to the server task
            if (sock_err < 0) {
                ESP_LOGE(FNAME, "Socket creation port %d FAILED with (%d): Abort ConfigureIntf", port, sock_err);
                return;
            }
            WifiEvent evt(AP_STARTED);
            xQueueSend(wifi_queue, &evt, 0);
        }
        ESP_LOGI(FNAME, "Sock creation successful port: %d socket: %d", port, sock->sock_hndl);
    }
}

int WifiApSta::Send(const char *msg, int &len, int port)
{
	if (port < 8880 || port > 8884 || !socket_server_task_pid) {
		ESP_LOGE(FNAME, "Invalid port: %d, should be between 8880 and 8884", port);
		return -1;
	}
	// ESP_LOGI(FNAME, "port %d to sent %d: bytes, %s", port, len, msg );
	int socket_num=port-8880;
	sock_ctrl_t *socks = _socks[socket_num];
	if( socks == nullptr ) {
		ESP_LOGI(FNAME, "no such sock avail");
		return -1;   // erratic port, socket unavail -> return, do not try ever again
	}
	// here we go
	bool sendOK = false;
	if ( ! socks->is_ap ) {
		// a STA socket
		if ( socks->sock_hndl > 0 && socks->_nr_peers == 1 ) {
			int num = send(socks->sock_hndl, msg, len, MSG_DONTWAIT);
			// ESP_LOGI(FNAME, "sta %d, num send %d", socks->port, num );
			if( num == len ){  // at least once we needed to sent the rest in one step for okay status
				sendOK = true;
				// ESP_LOGI(FNAME, "tcp send to client %d (port: %d), bytes %d/%d success", socks->sock_hndl, port, num, len );
			}
			else {
				ESP_LOGE(FNAME, "Send failed, socket %d", socks->sock_hndl );
				close(socks->sock_hndl);
				socks->sock_hndl = -1;
                socks->state = SocketState::DOWN;
                WifiEvent disconnect_evt(WifiEventType::STA_DISCONNECTED);
                xQueueSend(wifi_queue, &disconnect_evt, 0);
			}
		}
	}
	else {
		for (int c=0; c < MAX_CLIENTS; c++)  // iterate through all clients
		{
			peer_record &rec = socks->peers[c];
			if( rec.peer >= 0 ){
				int num = send(rec.peer, msg, len, MSG_DONTWAIT);
				// ESP_LOGI(FNAME, "client %d, num send %d", rec.client, num );
				if( num == len ){  // at least once we needed to sent the rest in one step for okay status
					sendOK = true;
					rec.retries = 0;
					// ESP_LOGI(FNAME, "tcp send to client %d (port: %d), bytes %d success", rec.peer, port, num );
				}
				else {
					ESP_LOGW(FNAME, "tcp send to  %d (port: %d), %d retries", rec.peer, port, rec.retries );
				}
			}
		}
	}
	socks->alive = sendOK; // if at least one client works, we say wifi is okay -> blue symbol
    return 0; // we do not want to trigger retries for WiFi
}

static esp_netif_t* wifi_config_sta(const char* staid) {
    ESP_LOGV(FNAME, "now esp_netif_create_default_wifi_sta");
    esp_netif_t* esp_netif_sta = esp_netif_create_default_wifi_sta();

    if (staid != nullptr && staid[0] != '\0') {
        ESP_LOGI(FNAME, "Configuring WiFi in STA mode with SSID: %s", staid);

        wifi_config_t wifi_sta_config;
        memset(&wifi_sta_config, 0, sizeof(wifi_sta_config));
        wifi_sta_config.sta.scan_method = WIFI_FAST_SCAN;  // stops scan after finding the SSID
        wifi_sta_config.sta.channel = 6;                   // a hint
        wifi_sta_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        wifi_sta_config.sta.bssid_set = false; // do not set bssid
        strcpy((char*)wifi_sta_config.sta.ssid, staid);
        strcpy((char*)wifi_sta_config.sta.password, AP_PASSPHARSE);

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    }
    return esp_netif_sta;
}

static esp_netif_t* wifi_config_softap(uint8_t maxcon, const char* ssid) {
    ESP_LOGV(FNAME, "now esp_netif_create_default_wifi_ap");
    esp_netif_t* esp_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_ap_config;
    memset(&wifi_ap_config, 0, sizeof(wifi_ap_config));
    strcpy((char*)wifi_ap_config.ap.ssid, ssid);
    wifi_ap_config.ap.ssid_len = strlen((char*)wifi_ap_config.ap.ssid);
    strcpy((char*)wifi_ap_config.ap.password, AP_PASSPHARSE);
    wifi_ap_config.ap.channel = 6;
    wifi_ap_config.ap.max_connection = maxcon;
    wifi_ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_ap_config.ap.ssid_hidden = 0;
    wifi_ap_config.ap.beacon_interval = 100;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
    ESP_LOGI(FNAME, "wifi config SUCCESS. SSID:%s password:%s channel:%d", (char*)wifi_ap_config.ap.ssid, (char*)wifi_ap_config.ap.password,
             wifi_ap_config.ap.channel);

    return esp_netif_ap;
}

bool WifiApSta::initialize_wifi(bool ap_mode, int maxcon, const char* ssid) {
    bool ret = false;
    if (!netif_initialized) {
        netif_initialized = true;
        ESP_ERROR_CHECK(esp_netif_init());  // can only be called once
    }
    if (!_sta_netif && !_ap_netif) {
        // create both modes the very first time
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        ESP_LOGV(FNAME, "now esp_wifi_init");
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_LOGV(FNAME, "now esp_event_handler_instance_register");
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
            WIFI_EVENT_HANDLER::wifi_event_handler, NULL, &_wifi_evnt_handler));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
            WIFI_EVENT_HANDLER::wifi_event_handler, NULL, &_ip_evnt_handler));

        if (!ap_mode) {
        // station config
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        _sta_netif = wifi_config_sta(ssid);
        ret = true; // some thing new
        }
        else {
            // access point config
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            _ap_netif = wifi_config_softap(maxcon, ssid);
            ret = true; // some thing new
        }

        // ESP_LOGV(FNAME,"now esp_wifi_set_storage");
        // ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        // For further testing, may improve wifi range
        // ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR ));

        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(int(wifi_max_power.get() * 80.0 / 100.0)));
        // start only once, even if we add the other mode later
        ESP_ERROR_CHECK(esp_wifi_start());
    }
    else {
        // we are already configured in one mode, now add the other mode if needed
        ESP_LOGI(FNAME, "WiFi already initialized, now add %s mode if needed", ap_mode ? "AP" : "STA");
    
        if (!ap_mode && !_sta_netif) {
            _sta_netif = wifi_config_sta(ssid);
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        }
        else if (ap_mode && !_ap_netif) {
            _ap_netif = wifi_config_softap(maxcon, ssid);
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        }
    }

    return ret; // s.th. new configured
}

void WifiApSta::client_reconnect()
{
	ESP_LOGI(FNAME,"wifi_reconnect()");
	wifi_config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.sta.channel = 6; // a hint
	sprintf( (char *)cfg.sta.ssid, "%s%d", SSID_PREFIX, (int)master_xcvario.get() );
	strcpy((char *)cfg.sta.password, AP_PASSPHARSE);

	ESP_ERROR_CHECK(esp_wifi_disconnect());
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
	ESP_ERROR_CHECK(esp_wifi_connect());
	// this will trigger the IP_EVENT_STA_GOT_IP event
}
