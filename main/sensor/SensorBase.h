/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/SetupNG.h"
#include "Filters.h"
#include "protocol/Clock.h"
// #include "logdef.h"

#include <type_traits>
#include <cstdint>
#include <cstring>  // for memcpy

//
// Memory-optimized fixed-size history buffer for sensors.
// 
// Uses a fixed-capacity circular buffer storing only values.
// 
// Only the timestamp of the latest reading is stored explicitly.
// Previous timestamps are reconstructed backwards using the fixed interval.
// 
// - Timestamps in uint32_t milliseconds since boot (>1000h before roll-over).
// - Capacity is computed to cover N seconds at the sensor's update rate.
// 

constexpr int SENSOR_HISTORY_DURATION_MS = 50000;  // milliseconds

enum SensorId : uint8_t;

template <typename T>
class FixedSensorHistory {
public:
    FixedSensorHistory() = delete;
    explicit FixedSensorHistory(T* buf, size_t cap) : 
        _capacity(cap), _head(0), _full(false), _heap_alloced(false), _buffer(buf) {
        if ( buf == nullptr ) {
            _buffer = (T*)malloc(cap + 4);
            _heap_alloced = true;
        }
        *_buffer = T{};
    }
    ~FixedSensorHistory() {
        if ( _heap_alloced ) free(_buffer);
    }
    void push(const T& value) {
        int nxt = (_head + 1) % _capacity;
        _buffer[nxt] = value;
        _head = nxt;
        if (_head == 0) { _full = true; }
    }
    int getCapacity() const {
        return _capacity;
    }
    int level() const {
        return _full ? _capacity : _head;
    }
    T getHead() const {
        return _buffer[_head];
    }
    T* getHeadPtr() const {
        return &_buffer[_head];
    }
    int getHeadIdx() const {
        return _head;
    }
    void reset() {
        _head = 0;
        _full = false;
        std::memset(_buffer, 0, sizeof(T) * _capacity);
    }
    T operator[] (int index) const { // Always count from head backwards
        return _buffer[wrap_back(index)];
    }
    
private:
    inline int wrap_back(int offset) const {
        return (_head + _capacity - (offset % _capacity)) % _capacity;
    }
    int    _capacity;
    int    _head;       ///< Index of the next write position (also count when full)
    uint8_t _full :1;   ///< Whether the buffer has wrapped around
    uint8_t _heap_alloced :1; ///< Whether the buffer was heap allocated

    alignas(T) T* _buffer;
};


//
// Sensor template and base class using fixed-size history
//
class SensorBase {
public:
    SensorBase(int ums);
    virtual ~SensorBase();

    virtual const char* name() const = 0;
    virtual bool probe() = 0;
    virtual bool setup() = 0;
    virtual bool update(uint32_t now_ms) = 0;
    virtual void postProcess() {};
    virtual bool isResting() const { return false; } // whether the sensor is in a calm state
    int getDutyCycle() const { return _update_interval_ms; }
    float getDutyCycleS() const { return (float)_update_interval_ms / 1000.0f; }
    int getLastObservationTime() const { return _last_update_time_ms + _latency_ms; }
    inline int getLastUpdateTimeMs() const { return _last_update_time_ms; }
    inline int getLatency() const { return _latency_ms; }
    inline SensorId getId() const { return _id; }

protected:
    uint32_t _update_interval_ms;  ///< Expected update interval
    uint32_t _latency_ms;          ///< Sensor conversion/acquisition latency
    uint32_t _last_update_time_ms; ///< Time the update got registered
    SensorId _id; /// SensorId as integer
};

template <typename T>
class SensorTP : public SensorBase {
public:
    SensorTP() = delete;
    SensorTP(void *buf, uint32_t ums) :
        SensorBase(ums),
        _history((T*)buf, HistoryCapacity(ums))
    {
        if constexpr (std::is_same_v<T, float>) { // only for float types
            _invalid = NAN;
        }
    }
    virtual ~SensorTP() {
    }
    void setNVSVar( SetupNG<float> *nvsvar ) {
        _nvsvar = nvsvar;
    }
    void setFilter( FilterItf<T>* filter ) {
        _filter = filter;
    }
    // read current value from sensor hardware
    virtual bool doRead(T &val) = 0;
    // optional: diagnostic info
    // virtual bool healthy() const { return true; }

    // Call this periodically from main loop or task.
    // returns true if new data was read.
    bool update(uint32_t now_ms) override {
        if ((now_ms - _last_update_time_ms) < _update_interval_ms) {
            return false;
        }

        // Read sensor && store in history
        T value;
        if (doRead(value)) {
            // Publish on black board NVS variable if linked
            pushAndPublish(value, now_ms);
        } else {
            // ESP_LOGE(FNAME, "Sensor %s read NAN", name());
            pushToHistory(_invalid, now_ms);
        }

        return true;
    }

    // The sensor bypass to fill the history directly (e.g. from group read)
    void pushToHistory(const T& value, uint32_t now_ms) {
        _last_update_time_ms = now_ms;
        _history.push(value);
    }
    // Publish on black board NVS variable
    void publishNVS() {
        if constexpr (std::is_same_v<T, float>) { // only for float types
            if (_nvsvar) {
                float fval = _history.getHead();
                if ( _filter ) {
                    fval = _filter->filter(fval);
                }
                _nvsvar->set(fval);
                _processed = fval;
            }
        }
    }
    inline void pushAndPublish(const T& value, uint32_t now_ms) {
        pushToHistory(value, now_ms);
        publishNVS();
    }

    // get the integral over the las X milli seconds
    T getIntegral(int interval_ms) const {
        int count = getCount(interval_ms);
        T sum = T{};

        // Start from latest and go backwards
        for (int i = 0; i < count; ++i) {
            sum += _history[i];
        }
        return sum;
    }
    
    // get average of the last X milli seconds
    T getAVG(int interval_ms) {
        int count = getCount(interval_ms);
        T sum = T{};

        // Start from latest and go backwards
        for (int i = 0; i < count; ++i) {
            sum += _history[i];
        }
        // ESP_LOGI(FNAME, "getAVG: count=%d samples=%d", count, samples);
        if (count == 0) {
            return T{};
        }
        sum /= (float)count;
        return sum;
    }

    // get population variance of the last X milli seconds (Welford)
    T getVariance(int interval_ms) {
        int count = getCount(interval_ms);
        if (count <= 1) return T{};

        T mean = T{};
        T M2 = T{};
        float n = 0.f;

        // Welford online algorithm for numerical stability
        for (int i = 0; i < count; ++i) {
            T val = _history[i];

            n += 1.f;
            T delta = val - mean;
            mean += delta / n;
            T delta2 = val - mean;
            M2 += delta * delta2;
        }

        return M2 / (float)count;  // population variance
    }

    /**
     * @brief Retrieve full history (caller provides buffer).
     * @param out_buffer Buffer to fill (must be at least HistoryCapacity long).
     * @param out_size Filled with actual number of samples.
     */
    // void getHistory(typename FixedSensorHistory<T, HistoryCapacity>::Reading* out_buffer,
    //                 size_t& out_size) const {
    //     _history.getHistory(out_buffer, out_size);
    // }

    // /**
    //  * @brief Reconstruct full history with timestamps (oldest first).
    //  */
    // void getHistory(Reading* out_history, size_t& out_size) const {
    //     size_t count = size();
    //     if (count == 0) {
    //         out_size = 0;
    //         return;
    //     }

    //     out_size = count;

    //     // Start from latest and go backwards
    //     uint32_t ts = last_timestamp_ms_;
    //     size_t idx = (head_ == 0) ? (Capacity - 1) : (head_ - 1);

    //     for (size_t i = 0; i < count; ++i) {
    //         out_history[i] = {ts, buffer_[idx]};
    //         if (ts >= interval_ms_) {
    //             ts -= interval_ms_;
    //         } else {
    //             ts = 0;  // Underflow protection (rare)
    //         }

    //         if (idx == 0) {
    //             idx = Capacity - 1;
    //         } else {
    //             --idx;
    //         }
    //     }

    //     // Reverse to have oldest first
    //     for (size_t i = 0; i < count / 2; ++i) {
    //         size_t j = count - 1 - i;
    //         Reading temp = out_history[i];
    //         out_history[i] = out_history[j];
    //         out_history[j] = temp;
    //     }
    // }

    // Get latest processed value (after filtering and NVS publish)
    T get() const {
        return _processed;
    }
    const T& getRef() const {
        return _processed;
    }

    // Get latest reading directly.
    T getHead() const {
        return _history.getHead();
    }
    T getHeadFiltered() const {
        if ( _filter ) {
            return _filter->get();
        }
        return _history.getHead();
    }
    T* getHeadPtr() const {
        return _history.getHeadPtr();
    }
    bool getHeadValid() const {
        return _history.level() > 0 && (_last_update_time_ms + _update_interval_ms * 2 > Clock::getMillis());
    }
    bool isValid(T val) const {
        if ( val != _invalid ) {
            return true;
        }
        return false;
    }
    int getFirstUpdateTimeMs() const { 
        int count = _history.level();
        return _last_update_time_ms - (count - 1) * _update_interval_ms;
    }

    // simple predictor to implement an innovation gate
    T predict() const {
        return _history.getHead() + (_history.getHead() - _history[1]);
    }
    bool accept(T x_meas, T max_jump) const {
        if constexpr (std::is_same_v<T, float>) { // only for float types
            if (!getHeadValid()) {
                return true;
            }

            float x_pred = _history.getHead() + (_history.getHead() - _history[1]);
            float resid  = x_meas - x_pred;
            if (fabsf(resid) > max_jump) {
                return false; // outlier
            }
        }
        return true;
    }

protected:
    // Capacity = ceil(5000 / _update_interval_ms)
    static constexpr size_t HistoryCapacity(uint32_t ums) { return (SENSOR_HISTORY_DURATION_MS + ums - 1) / ums; }
    // time window to sample count
    int getCount(int interval_ms) const {
        uint32_t cutoff_time = Clock::getMillis() - interval_ms;
        return std::min((_last_update_time_ms - cutoff_time) / _update_interval_ms, (uint32_t)_history.level());
    }

    FixedSensorHistory<T> _history;
    SetupNG<float> *_nvsvar = nullptr; ///< Optional link to NVS variable for sync etc.
    FilterItf<T>*   _filter = nullptr; ///< Optional filter plugin, ownership not handled here
    T               _invalid = T{};    ///< Invalid value representation
    T               _processed = T{};  ///< Last valid value as published on the black board
};

