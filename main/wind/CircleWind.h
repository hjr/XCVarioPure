/***********************************************************************
 **
 **   Copyright (c):  2002       by André Somers,
 **	                  2002-2006  Eckhard Völlm
 **                   2006-      by Axel Pauli
 **                   2021-      Eckhard Völlm
 **
 **
 ***********************************************************************/

/**
 * \class CircleWind
 *
 * \author André Somers, Eckhard Völlm, Axel Pauli
 *
 * \brief wind analyzer
 *
 * The wind analyzer processes the list of flight samples looking
 * for wind speed and direction.
 *
 * \date 2002-2021
 */

#pragma once

#include "sensor/Filters.h"

#include "vector.h"
#include <cmath>
#include <list>

typedef enum e_circling {
  undefined,
  straight,
  circlingL,
  circlingR
} t_circling;


class CircleWind {
public:
  CircleWind();
  ~CircleWind();
  void tick();

  // Call for wind measurement result. The result is included in wind,
  // the jitter of the measurement (1-5; 1 is bad, 5 is excellent) in jitter.
  void getWind(Vector &wind, int &qual) const {
    wind = result;
    qual = jitter;
  };

  // Called if the flight mode changes
  void newFlightMode(t_circling newMode);
  t_circling getFlightMode() const { return flightMode; };

  // Calculate flightmode from heading diff if circling is left or right
  void calcFlightMode(rad_t diff, mps_t speed);

  // Called if a new sample is available in the sample list.
  void setNewSample(Vector flarm_vector) { flarmVec = flarm_vector; };
  void newSample();

  void restartCycle(bool clean);

  // Called if a new satellite constellation has been detected.
  void newConstellation(int numSat);

  // Called, if the GPS status has changed.
  void setGpsStatus(bool newStatus);

  void newWind(rad_t angle, mps_t speed);

  static bool getWind(int16_t *dir, mps_t *speed);

  float getNumCircles() const { return circleCount + (circleArc / 360.0); }
  int getSatCnt() const { return satCnt; }
  bool getGpsStatus() const { return gpsStatus; }
  degree_t getAngleDeg() { return result.getAngleDeg(); }
  mps_t getSpeed() { return result.getSpeed(); }
  int getAge() const { return _age; }
  static void resetAge();
  int getQuality() const { return rint(100.0 - jitter); } // 0..100 %
  const char *getStatus() const { return status; }
  const char *getFlightModeStr() const;

private:
  void _calcWind();
  int circleCount = 0; // we are counting the number of circles, the first onces are
                   // probably not very round
  bool circleLeft = false; // true=left, false=right
  rad_t circleArc = 0.; // Arc of current flown circle
  rad_t lastHeading = 0.;   // Last processed heading
  int satCnt = 0;
  static constexpr int minSatCnt = 5;
  t_circling circlingMode = undefined;
  int gpsStatus = false;
  Vector minVector;
  Vector maxVector;
  Vector result;
  float jitter = 0;
  t_circling flightMode = undefined;
  static int16_t _age;
  const char *status;
  LowPassFilter _lp_headdiff; // we filter the heading a bit to get a more stable circle detection
  std::list<Vector> windVectors;
  uint8_t turn_left;
  uint8_t turn_right;
  uint8_t fly_straight;
  float lastWindDir;
  mps_t lastWindSpeed;
  Vector flarmVec;
};

extern CircleWind *circleWind;
