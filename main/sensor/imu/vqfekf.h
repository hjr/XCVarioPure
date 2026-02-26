
/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/Units.h"
#include "math/vector_3d.h"
#include "../Filters.h"

class BiasEstimatorEKF {
public:
    BiasEstimatorEKF();
    bool detectRest(const vector_f& gyro, const vector_f& accel, second_t dt);
    void update(const vector_f& gyro_meas, second_t dt);
    inline const vector_f& getBias() const { return biasEstimate; }
    inline float getRestDuration() const { return restTimer; }
    void reset();

private:
    // VQF like Rest-Detection
    bool isResting = false;
    float restTimer = 0.0f;          // Sekunden, seit letzter Bewegung
    // float gyrNormLP = 0.0f;          // Low-Pass gefilterte Gyro-Norm
    float accNormLP = 0.0f;          // Low-Pass gefilterte Acc-Norm (Abweichung von g)
    
    // Separate Bias-Kalman-Filter (nur für Gyro-Bias, 3 Zustände)
    vector_f biasEstimate = {};      // Aktueller geschätzter Gyro-Bias
    vector_f biasP = {};             // Kovarianz-Diagonal (vereinfacht, pro Achse unabhängig)
    float biasQ = 1e-6f;             // Prozessrauschen (sehr klein)
    float restR = 1e-5f;             // Messrauschen bei Rest (sehr klein → starkes Vertrauen)
    float motionR = 1e-2f;           // Messrauschen bei Motion (groß → schwaches Vertrauen)

    // Low-Pass-Koeffizienten (tau ≈ 0.5–1 s)
    float tauLP = 0.8f;              // Zeitkonstante in Sekunden
};
