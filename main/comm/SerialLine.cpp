/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SerialLine.h"

#include "comm/DataLink.h"
#include "math/Floats.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <soc/uart_reg.h>

#include <algorithm>


constexpr int BUF_LEN = 128;
constexpr int UARTRXFIFO_LEN = 256;
constexpr int UARTTXFIFO_LEN = 1024;
constexpr int UARTEVENTQ_LEN = 10;

static constexpr std::array<int, 7> baud_rate_table = { 2400, 4800, 9600, 19200, 38400, 57600, 115200 };  // stay compatible with existing setup's
static constexpr std::array<e_baud, 7> scan_order = { BAUD_19200, BAUD_38400, BAUD_57600, BAUD_115200, BAUD_9600, BAUD_4800, BAUD_2400 };
constexpr int GARBAGE_LIMIT = 2 * ProtocolItf::MAX_LEN;
constexpr int STALE_GARBAGE = 32;
constexpr TickType_t STALE_TIME = pdMS_TO_TICKS(500);
constexpr TickType_t LOCKED_STALE_TIME = pdMS_TO_TICKS(2000);
constexpr TickType_t QUIET_RESET = pdMS_TO_TICKS(2000);

// enumeration is: SM_FLARM, SM_RADIO, SM_XCTNAV_S3, SM_OPENVARIO
static constexpr std::array<t_serial_cfg, NR_OF_SER_PROFILES> sm_serial_config = {{
    // baud,      polarity,   pinswp, tx enable
    {BAUD_19200, RS232_TTL, false, true},
    {BAUD_9600, RS232_TTL, false, true},
    {BAUD_38400, RS232_TTL, true, true},
    {BAUD_19200, RS232_TTL, false, true}}};

static constexpr std::array<std::string_view, 3> MEMOS = {"S0", "S1", "S2"};

// connection to nvs setup variables
static constexpr std::array<t_serial_nvs_setup, 3> uart_setup = {{
    {nullptr, nullptr, nullptr, nullptr, nullptr},
    {&serial1_speed, &serial1_ttl_signals, &serial1_pin_swap, &serial1_tx_enable, &serial1_baud_mode},
    {&serial2_speed, &serial2_ttl_signals, &serial2_pin_swap, &serial2_tx_enable, &serial2_baud_mode}}};

static constexpr uart_event_t stop_trigger = {UART_EVENT_MAX, 0, false};

static int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

void BaudProbe::feed(const char *c, int len)
{
    for (; len > 0; c++, len--) {
        unsigned char ch = *c;
        if (ch != '\r' && ch != '\n') {
            _garbage++;
        }
        if (ch == '$' || ch == '!') {
            _state = BODY;
            _crc = 0;
            _len = 0;
            continue;
        }
        switch (_state) {
        case BODY:
            if (ch == '*') {
                _state = CRC1;
            } else if (ch < 0x20 || ch > 0x7e || ++_len > ProtocolItf::MAX_LEN) {
                _state = IDLE;
            } else {
                _crc ^= ch;
            }
            break;
        case CRC1: {
            int hi = hexValue(ch);
            if (hi < 0) {
                _state = IDLE;
                break;
            }
            _crc ^= hi << 4;
            _state = CRC2;
            break;
        }
        case CRC2: {
            int lo = hexValue(ch);
            if (lo >= 0 && (_crc ^ lo) == 0) {
                _good++;
                _garbage = 0;
            }
            _state = IDLE;
            break;
        }
        default:
            break;
        }
    }
}

// Thread body to handle UART events
void uartTask(SerialLine *s) {
    uart_event_t event;
    char rx_buf[BUF_LEN + 1];
    uart_port_t un = s->uart_nr;

    ESP_LOGI(FNAME, "IoSerTask UART%d", un);

    while (true) {
        // sleep until the UART gives us something to do
        BaseType_t ret = xQueueReceive((QueueHandle_t)s->_event_queue, &event, pdMS_TO_TICKS(10000));
        // fetch the data link
        DataLink* dltarget = s->_dlink.begin()->link; // only the first entry is used here
        if (ret != pdTRUE) {
            // time-out, propagate as empty message,
            // or no data link
            if (dltarget) {
                dltarget->process(nullptr, 0);
            }
            continue;
        }

        if (event.type == UART_EVENT_MAX) {
            ESP_LOGI(FNAME, "Terminate UART%d task", un);
            break;
        } // termination signal

        switch (event.type) {
        case UART_PATTERN_DET:
            // ESP_LOGI(FNAME, "Pattern detected on UART%d", un);
        case UART_DATA: {
            int count = uart_read_bytes(un, rx_buf, BUF_LEN, 0);
            if (count > 0) {
                if (s->baudDetecting()) {
                    s->detectBaud(rx_buf, count);
                }
                if (dltarget) {
                    rx_buf[count] = '\0';
                    // ESP_LOGI(FNAME, "Data received from UART%d: %dc", un, count);
                    dltarget->process(rx_buf, count); // SX interfaces do have only one data link
                }
            }
            break;
        }
        case UART_BREAK:
            s->break_count++;
            if (s->baudDetecting()) {
                s->detectBaud(nullptr, 0);
            }
            // ESP_LOGI(FNAME, "Break detected on UART%d", un);
            break;
        case UART_BUFFER_FULL:
            // ESP_LOGI(FNAME, "Full detected on UART%d", un);
            break;
        case UART_FIFO_OVF:
            // ESP_LOGI(FNAME, "OVF detected on UART%d", un);
            break;
        case UART_DATA_BREAK:
            // ESP_LOGI(FNAME, "DataBreak detected on UART%d ", un);
            break;
        default:
            // ESP_LOGI(FNAME, "UART%d caught event %d", un, event.type);
            break;
        }
    }
    vTaskDelete(nullptr);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// SerialLine
///////////////////////////////////////////////////////////////////////////////////////////////////////
SerialLine::SerialLine(uart_port_t uart, gpio_num_t rx, gpio_num_t tx) : 
    InterfaceCtrl(true), 
    cfg(0), uart_nr(uart), 
    tx_gpio(tx), rx_gpio(rx), 
    _intfid(InterfaceId((int)S0_RS232 + (int)uart)), 
    _id_memo(MEMOS[uart].data()),
    _setup(uart_setup[uart])
{
    loadSetupDefaults();
    ESP_LOGI(FNAME, "CONSTR. UART:%d (new) RX:%d TX:%d baud:%d pol:%d swap:%d tx:%d", uart_nr, rx_gpio, tx_gpio, baud_rate_table[cfg.baud], cfg.polarity,
             cfg.pin_swp, cfg.tx_ena);
}

SerialLine::~SerialLine()
{
    stop();
}

// 0,1,2,3..: load profile
void SerialLine::ConfigureIntf(int profile) {
    ESP_LOGI(FNAME, "configure UART S%d: %d", uart_nr, profile);
    if (!uart_is_driver_installed(uart_nr)) {
        if (profile >= 0) {
            loadProfile(e_profile(profile));
        }
        start();
    } else {
        if (profile >= 0 && profile < NR_OF_SER_PROFILES) {
            const t_serial_cfg *newcfg = &sm_serial_config[profile];
            if (cfg.baud != newcfg->baud) {
                cfg.baud = newcfg->baud;
                applyBaud();
            }
            if (cfg.polarity != newcfg->polarity) {
                cfg.polarity = newcfg->polarity;
                applyLineInverse();
            }
            if (cfg.tx_ena != newcfg->tx_ena || cfg.pin_swp != newcfg->pin_swp) {
                cfg.tx_ena = newcfg->tx_ena;
                cfg.pin_swp = newcfg->pin_swp;
                applyPins();
            }
        }
    }
}

// returns 0: sucess; or retry wait time in msec
int SerialLine::Send(const char *msg, int &len, int port)
{
    int count = uart_write_bytes(uart_nr, msg, len);
    ESP_LOGI(FNAME, "Send UART%d, len %d/%d", uart_nr, len, count);
    if (count == len) {
        return 0;
    }
    count = (count >= 0) ? count : 0;                                        // enforce count >= 0
    int dur = fast_iroundf((len - count) * 12000.0 / baud_rate_table[cfg.baud] + 1); // [msec] ; byte/sec = baudrate / 12 bits/char
    ESP_LOGI(FNAME, "Send UART%d, ETA for free buf in %d msec", uart_nr, dur);
    len = count; // buffered chars
    return dur;  // rough ETA for telegram of len bytes
}

// void SerialLine::flush(){
// 	uart_flush(uart_nr);
// }


void SerialLine::applyBaud()
{
    ESP_LOGI(FNAME, "setBaud UART%d baud idx %d: %d", uart_nr, cfg.baud, baud_rate_table[cfg.baud]);
    if (uart_is_driver_installed(uart_nr)) {
        uart_flush(uart_nr);
    }
    uart_set_baudrate(uart_nr, baud_rate_table[cfg.baud]);
    _rescan = true;
}

void SerialLine::setBaudMode(e_baud_mode mode)
{
    _baud_mode = mode;
    _setup.baud_mode->set(mode);
    _rescan = true;
    if (mode == BAUD_MODE_FIXED && cfg.baud != _setup.baud->get()) {
        cfg.baud = (e_baud)_setup.baud->get();
        applyBaud();
    }
}

// binary protocols and the binary XCV sync frames cannot be validated by the NMEA probe
bool SerialLine::baudDetectApplicable()
{
    DataLink *dl = _dlink.begin()->link;
    return dl && dl->getNmea() && !dl->isBinActive() && !dl->hasProtocol(XCVSYNC_P);
}

// rx == nullptr: a line break
void SerialLine::detectBaud(const char *rx, int len)
{
    TickType_t now = xTaskGetTickCount();
    bool rescan = _rescan.exchange(false);
    if (rescan) {
        _baud_locked = false;
    }
    // noise between quiet periods must not add up to a rate change
    if (rescan || now - _last_rx > QUIET_RESET) {
        _probe.reset();
    }
    _last_rx = now;
    int good = _probe.good();
    bool clean = _probe.garbage() == 0;
    if (rx) {
        _probe.feed(rx, len);
    } else {
        _probe.noise();
    }
    if (clean || _probe.good() > good) {
        _garbage_since = now;
    }
    TickType_t since = now - _garbage_since;
    int garbage = _probe.garbage();
    bool leave = _baud_locked
        // dropped bytes break single sentences, a wrong rate leaves none valid
        ? garbage >= GARBAGE_LIMIT && since > LOCKED_STALE_TIME
        // a slower rate than the sender's sees only a few bytes of it
        : garbage >= GARBAGE_LIMIT || (garbage >= STALE_GARBAGE && since > STALE_TIME);
    if (leave) {
        if (_baud_locked) {
            ESP_LOGI(FNAME, "UART%d lost sync at %d", uart_nr, baud_rate_table[cfg.baud]);
            _baud_locked = false;
        }
        auto pos = std::find(scan_order.begin(), scan_order.end(), cfg.baud) - scan_order.begin();
        cfg.baud = scan_order[(pos + 1) % scan_order.size()];
        applyBaud();
    } else if (!_baud_locked && _probe.good() >= 2) {
        ESP_LOGI(FNAME, "UART%d locked at %d", uart_nr, baud_rate_table[cfg.baud]);
        _baud_locked = true;
        _setup.baud->set(cfg.baud);
        if (_baud_mode == BAUD_MODE_DETECT_ONCE) {
            setBaudMode(BAUD_MODE_FIXED);
        }
    }
}

// configures ESP32 matrix, here also the line polartity (inverting) is handled
void SerialLine::applyPins()
{
    ESP_LOGI(FNAME, "setPinSwap UART%d swap:%d (0:NORM, 1:SWAPPED), rx:%d, tx:%d,", uart_nr, cfg.pin_swp, rx_gpio, tx_gpio);

    gpio_num_t tx_pin = cfg.pin_swp ? rx_gpio : tx_gpio;
    gpio_num_t rx_pin = cfg.pin_swp ? tx_gpio : rx_gpio;

    if (uart_is_driver_installed(uart_nr)) {
        // pause it
        uart_flush(uart_nr);
        uart_disable_rx_intr(uart_nr);
    }
    uart_set_pin(uart_nr, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (uart_is_driver_installed(uart_nr)) {
        // and restart it
        uart_enable_rx_intr(uart_nr);
    }

    if (!cfg.tx_ena) {
        // Overrule uart module and switch tx off if configured
        gpio_set_direction(tx_pin, GPIO_MODE_INPUT);
        gpio_pullup_dis(tx_pin);
    }
}

void SerialLine::applyLineInverse()
{
    ESP_LOGI(FNAME, "setLineInverse UART%d: %d (0: Norm, 1:Invers)", uart_nr, cfg.polarity);
    if (uart_is_driver_installed(uart_nr)) {
        uart_flush(uart_nr);
    }
    if (cfg.polarity) {
        uart_set_line_inverse(uart_nr, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
        gpio_pulldown_dis(tx_gpio);
        gpio_pulldown_dis(rx_gpio);
        gpio_pullup_en(tx_gpio);
        gpio_pullup_en(rx_gpio);

    } else {
        uart_set_line_inverse(uart_nr, 0);
        gpio_pulldown_en(tx_gpio);
        gpio_pulldown_en(rx_gpio);
        gpio_pullup_dis(tx_gpio);
        gpio_pullup_dis(rx_gpio);
    }
}

// set defaults according to given profile
void SerialLine::loadProfile(e_profile profile)
{
    ESP_LOGI(FNAME, "loadProfile: %d (0: Flarm, 1:Radio)", profile);
    cfg = sm_serial_config[profile];
    _setup.baud->set(cfg.baud);
    _setup.polarity->set(cfg.polarity);
    _setup.tx_ena->set(cfg.tx_ena);
    _setup.pin_swp->set(cfg.pin_swp);
}
// load setup defaults
void SerialLine::loadSetupDefaults()
{
    cfg.baud = (e_baud)_setup.baud->get();
    cfg.polarity = _setup.polarity->get();
    cfg.tx_ena = _setup.tx_ena->get();
    cfg.pin_swp = _setup.pin_swp->get();
    _baud_mode = (e_baud_mode)_setup.baud_mode->get();
    ESP_LOGI(FNAME, "load default: %d/%d/%d/%d", cfg.baud, cfg.polarity, cfg.tx_ena, cfg.pin_swp);
}

void SerialLine::start() {
    if (!uart_is_driver_installed(uart_nr)) {
        ESP_LOGI(FNAME, "start UART%d baud:%d, rx:%d, tx:%d, TTL:%d ", uart_nr, cfg.baud, rx_gpio, tx_gpio, cfg.polarity);

        // all the details and baudrate
        uart_config_t uart_config = {
            .baud_rate = baud_rate_table[cfg.baud],
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT,
            .flags = {.allow_pd = 0, .backup_before_sleep = 0},
        };
        uart_param_config(uart_nr, &uart_config);

        // line logic levels and pins
        applyLineInverse();
        applyPins();

        // Setup UART buffered IO with event queue
        // Install UART driver creating an event queue here
        uart_driver_install(uart_nr, UARTRXFIFO_LEN, UARTTXFIFO_LEN, UARTEVENTQ_LEN, (QueueHandle_t *)&_event_queue, ESP_INTR_FLAG_IRAM);

        // Set UART pattern detect function
        uart_enable_pattern_det_baud_intr(uart_nr, '\n', 1, 5, 0, 0);

        if (!_iotask) {
            char tmp[] = "S1rx";
            tmp[1] = '0' + uart_nr;
            xTaskCreate((TaskFunction_t)uartTask, tmp, 4000, this, 19, (tskTaskControlBlock **)&_iotask);
        }
        uart_intr_config_t intr_config = {.intr_enable_mask = UART_RXFIFO_TOUT_INT_ENA | UART_RXFIFO_OVF_INT_ENA | UART_AT_CMD_CHAR_DET_INT_ENA |
                                                              UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA,
                                          // | UART_TXFIFO_EMPTY_INT_ENA | UART_TX_BRK_IDLE_DONE_INT_ENA | UART_TX_BRK_DONE_INT_ENA | UART_TX_DONE_INT_ENA,
                                          .rx_timeout_thresh = 9,
                                          .txfifo_empty_intr_thresh = 0,
                                          .rxfifo_full_thresh = 60};
        uart_intr_config(uart_nr, &intr_config);
    }
}

void SerialLine::stop() {
    if (_iotask) {
        // Last chance to use the queue to terminate the receiver task
        xQueueSend((QueueHandle_t)_event_queue, (void *)&stop_trigger, (TickType_t)0);
    }
    _iotask = nullptr;

    if (uart_is_driver_installed(uart_nr)) {
        ESP_LOGI(FNAME, "UART %d end(), pins %d %d", uart_nr, tx_gpio, rx_gpio);
        uart_flush(uart_nr);
        uart_driver_delete(uart_nr);
    }
}

// Do a simple loop back test (before configuring the interface or starting the receiver task !)
bool SerialLine::selfTest(void)
{
    const char *test_str = "The quick brown fox jumps over the lazy dog";
    uint8_t rxbuf[BUF_LEN];

    // UART config
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {.allow_pd = 0, .backup_before_sleep = 0},
    };

    // Init UART driver (no real pins needed)
    ESP_ERROR_CHECK(uart_param_config(uart_nr, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(uart_nr, BUF_LEN*2, BUF_LEN*4, 0, NULL, 0));

    // activate internal loopback
    uart_set_loop_back(uart_nr, 1);

    // Send test message
    uart_write_bytes(uart_nr, test_str, strlen(test_str));

    // Wait for RX
    vTaskDelay(pdMS_TO_TICKS(50));

    int len = uart_read_bytes(uart_nr, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(100));
    bool ret = false;
    if (len > 0) {
        rxbuf[len] = 0;
        if (strcmp((char *)rxbuf, test_str) == 0) {
            ESP_LOGI(FNAME, "UART loopback test passed");
            ret = true;
        } else {
            ESP_LOGE(FNAME, "UART loopback mismatch: '%s'", rxbuf);
        }
    } else {
        ESP_LOGE(FNAME, "UART loopback read timeout");
    }
    _functional = ret; // release intf for use

    // Restore normal UART mode
    uart_set_loop_back(uart_nr, 0);

    uart_driver_delete(uart_nr);
    return ret;
}
