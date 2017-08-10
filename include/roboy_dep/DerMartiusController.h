/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/
#ifndef RALFDERCONTROLLER_H_
#define RALFDERCONTROLLER_H_

#include "roboy_dep/matrix.h"
#include "roboy_dep/ringbuffer.h"
#include "roboy_dep/inspectable.h"

#include <cmath>


//#pragma once

class DerMartiusController : public Inspectable {
private:
  matrix::Matrix M, C, CC, C_update, h, normmot, xCurrent, yCurrent, forces;
  RingBuffer<matrix::Matrix> x_buffer;
  RingBuffer<matrix::Matrix> x_raw_buffer;
  RingBuffer<matrix::Matrix> y_buffer;
  long int t;
  unsigned int number_motors;
  unsigned int number_sensors;
  unsigned int number_raw_sensors;

  int buffer_depth;
  bool useDelay;

  static double clip(double limit ,double x);
  static double g(double x);
  bool invDelayCoupling;

  void guide(matrix::Matrix& y); // guide controller by superimposing actions

public:
  int timedist;
  double urate;
  double initFeedbackStrength;
  int regularization;
  double synboost;
  double maxSpeed;
  double epsh;
  double springMult1;
  double pretension;
  double maxForce;
  int delay; // delay of delay sensors
  bool splitBrain; // each motor is independent
  double stepNo; // it is actually t, just as a double for exporting

  int guideType; // type of guidance
  int guideIdx; // param for guidance
  double guideAmpl; // param for guidance
  double guideFreq; // param for guidance

  bool learning; //enable/disable learning update

  void setInvDelayCoupling(bool);
  bool getInvDelayCoupling() { return invDelayCoupling; }

public:
  DerMartiusController(unsigned int number_motors,unsigned int number_sensors, bool useDelaySensors);
  ~DerMartiusController(){}
  matrix::Matrix update(const matrix::Matrix& sensor_values, const matrix::Matrix& force_values);

  void paramsToLogFile(); // add the current parameters to log file (call before plotengine init)
  bool storeParamsToFile(FILE* f) const;
  bool restoreParamsFromFile(FILE* f);

  bool store(FILE* f) const;
  bool restore(FILE* f);
  bool storeToFile(const char* filename) const;
  bool restoreFromFile(const char* filename);
  matrix::Matrix getC();
};

#endif
