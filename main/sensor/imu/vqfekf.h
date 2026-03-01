
/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/vector_3d.h"

class BiasEstimatorEKF {
public:
    BiasEstimatorEKF();
    void update(const vector_f& gyro_meas, bool isResting);
    inline const vector_f& getBias() const { return biasEstimate; }
    void reset();

private:
    // Separate Bias-Kalman-Filter (nur für Gyro-Bias, 3 Zustände)
    vector_f biasEstimate = {};      // Aktueller geschätzter Gyro-Bias
    vector_f biasP = {};             // Kovarianz-Diagonal (vereinfacht, pro Achse unabhängig)
    float biasQ = 1e-6f;             // Prozessrauschen (sehr klein)
    float restR = 1e-5f;             // Messrauschen bei Rest (sehr klein → starkes Vertrauen)
    float motionR = 1e-2f;           // Messrauschen bei Motion (groß → schwaches Vertrauen)
};
