/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "vqfekf.h"
#include <cmath>

#include "GyroMPU6050.h"
#include "logdef.h"

BiasEstimatorEKF::BiasEstimatorEKF() :
    biasP{1e-3f, 1e-3f, 1e-3f} // initial covariance (relativ hoch, da Anfangsunsicherheit) 
{
}


// rest - detection
bool BiasEstimatorEKF::detectRest(const vector_f& gyro_raw, const vector_f& accel_raw, second_t dt) {
    // gyro variance and acc deviation from 1g
    vector_f gyrVar = gyroSensor->getVariance(1000);
    float accDev = std::fabsf(accel_raw.get_norm() * Units::g0 - Units::g0);
    
    // low pass filter for acc deviation
    accNormLP = accNormLP + (dt / tauLP) * (accDev - accNormLP);
    // ESP_LOGI(FNAME, "rest detection: gyrVar=(%f, %f, %f) accDev=%f accNormLP=%f, dt=%f", gyrVar.x, gyrVar.y, gyrVar.z, accDev, accNormLP, dt);
    
    // thresholds (VQF-typical)
    constexpr float thGyr = Units::deg_to_rad(0.2f);   // °/s
    constexpr float thAcc = 0.7f;             // m/s² deviation from 1g
    
    if (gyrVar.x < thGyr && gyrVar.y < thGyr && gyrVar.z < thGyr && accNormLP < thAcc) {
        restTimer += dt;
        if (restTimer > 1.5f) {
             // min. 1.5–3 sec below threshold → consider as rest
            isResting = true;
        }
    } else {
        restTimer = 0.0f;
        isResting = false;
    }
    return isResting;
}

// bias filter update
void BiasEstimatorEKF::update(const vector_f& gyro_meas, second_t dt) {
    vector_f meas; // gyro reading
    float R;    // current measurement noise (depends on rest/motion)
    
    if (isResting) {
        // trust measurement is mostly bias
        meas = gyro_meas;  // directly the measurement
        R = restR;
    } else {
        // During motion: No direct measurement → only process update (no measurement update!)
        // or weak update from your main EKF
        meas = {0,0,0};    // dummy
        R = motionR;       // large -> almost no update
    }
    
    // prediction
    biasP += dt * biasQ;

    // update
    if (isResting) {
        vector_f innov = meas - biasEstimate;
    
        // Kalman gain
        vector_f K     = biasP / (biasP + R);
    
        // state- und covariance update
        biasEstimate += K * innov;
        biasP *= vector_f(1.f, 1.f, 1.f) - K;
    }
}

void BiasEstimatorEKF::reset() {
    isResting = false;
    restTimer = 0.0f;
    accNormLP = 0.0f;
    biasEstimate = {};
    biasP = {1e-3f, 1e-3f, 1e-3f};
}