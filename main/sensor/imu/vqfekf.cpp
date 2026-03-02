/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "vqfekf.h"


BiasEstimatorEKF::BiasEstimatorEKF() :
    biasP{1e-3f, 1e-3f, 1e-3f} // initial covariance (relativ hoch, da Anfangsunsicherheit) 
{
}


// bias filter update
void BiasEstimatorEKF::update(const vector_f& gyro_meas, bool isResting) {
    constexpr float dt = 0.1f;
    vector_f meas; // gyro reading
    float R;    // current measurement noise (depends on rest/motion)
    
    if (isResting) {
        // trust measurement is mostly bias
        meas = gyro_meas;  // directly the measurement
        R = restR;
    } else {
        // during motion: No direct measurement → only process update (no measurement update!)
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
    biasEstimate = {};
    biasP = {1e-3f, 1e-3f, 1e-3f};
}