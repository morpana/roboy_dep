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

#include "roboy_dep/DerMartiusController.h"
#include "roboy_dep/inspectable.h"
#include <roboy_dep/DEP.hpp>

using namespace matrix;
using namespace std;

double DerMartiusController::g(double x){
  return tanh(x);
}

/// clipping function for mapP
double DerMartiusController::clip(double limit,double x){
  if(x<-limit) return -limit; else if(x>limit) return limit; else return x;
}

DerMartiusController::DerMartiusController(unsigned int number_mot,unsigned int number_sens, bool useDelaySensors)
  :Inspectable("Ralf+Georg")
{
  // Initialisierungen:
  timedist = 8;
  urate = 0.01;  // 0.01
  initFeedbackStrength = 0;
  regularization = 10;
  synboost = 1.7;
  maxSpeed = 1;
  epsh = 0.0;
  buffer_depth = 200;
  pretension = 5;
  maxForce   = 300;
  springMult1 = 1;
  delay       = 50;
  guideType   = 0;
  guideIdx    = 0;
  guideAmpl   = 0.3;
  guideFreq   = 0.5;

  useDelay    = useDelaySensors;
  number_motors = number_mot;
  number_raw_sensors = number_sens;
  number_sensors = useDelaySensors ? number_sens * 2 : number_sens;
  splitBrain=false;

  M = Matrix(number_motors, number_sensors);
  //       M.set(number_motors, number_sensors);
  C = Matrix(number_motors, number_sensors);
  //       C.set(number_motors, number_sensors);
  CC = Matrix(number_motors, number_sensors);
  //       CC.set(number_motors, number_sensors);
  C_update = Matrix(number_motors, number_sensors);
  //       C_update.set(number_motors, number_sensors);
  h = Matrix(number_motors, 1);
  //       h.set(number_motors, 1);
  normmot = Matrix(number_motors, 1);
  //       normmot.set(number_motors, 1);
  M.toId(); // set a to identity matrix;

  invDelayCoupling=false;
  setInvDelayCoupling(true); // this sets the right M-matrix if delay sensors are used

  C.toId(); // set C to identity matrix;
  C*=initFeedbackStrength; //default: initFeedbackStrength = 0;
  C_update.toId();
  C_update*= initFeedbackStrength;
  CC.toZero();
  h.toZero();
  forces=Matrix(number_raw_sensors,1);
  xCurrent=Matrix(number_sensors,1);
  yCurrent=Matrix(number_motors,1);

  x_buffer.init(buffer_depth, Matrix(number_sensors,1));
  y_buffer.init(buffer_depth, Matrix(number_motors,1));
  x_raw_buffer.init(buffer_depth, Matrix(number_raw_sensors,1));

  addInspectableMatrix("h", &h, false,   "acting controller bias");
  addInspectableMatrix("CC", &CC, false, "acting controller matrix");

  addInspectableMatrix("forces", &forces, false, "force values");
  addInspectableMatrix("x", &xCurrent, false, "sensor");
  addInspectableMatrix("y", &yCurrent, false, "motor");

  addInspectableValue("step", &stepNo, "step number");
  t = 0;
  stepNo=0;
}

void DerMartiusController::paramsToLogFile(){
  addInfoLine("timedist      : " + itos(timedist      ));
  addInfoLine("urate         : " + ftos(urate         ));
  addInfoLine("regularization: " + ftos(regularization));
  addInfoLine("synboost      : " + ftos(synboost      ));
  addInfoLine("epsh          : " + ftos(epsh          ));
  addInfoLine("pretension    : " + ftos(pretension    ));
  addInfoLine("maxForce      : " + ftos(maxForce      ));
  addInfoLine("springMult1   : " + ftos(springMult1   ));
  addInfoLine("delay         : " + itos(delay         ));
  addInfoLine("splitBrain    : " + itos(splitBrain    ));
}
bool DerMartiusController::storeParamsToFile(FILE* f) const {
  fprintf(f,"timedist      : %i\n",  timedist      );
  fprintf(f,"urate         : %f\n",  urate         );
  fprintf(f,"regularization: %i\n",  regularization);
  fprintf(f,"synboost      : %f\n",  synboost      );
  fprintf(f,"epsh          : %f\n",  epsh          );
  fprintf(f,"pretension    : %f\n",  pretension    );
  fprintf(f,"maxForce      : %f\n",  maxForce      );
  fprintf(f,"springMult1   : %f\n",  springMult1   );
  fprintf(f,"delay         : %i\n",  delay         );
  fprintf(f,"splitBrain    : %i\n",  splitBrain    );
  return true;
}
bool DerMartiusController::restoreParamsFromFile(FILE* f){
	  fscanf(f,"timedist      : %i",  &timedist      );
	  fscanf(f,"urate         : %lf",  &urate         );
	  fscanf(f,"regularization: %i",  &regularization);
	  fscanf(f,"synboost      : %lf",  &synboost      );
	  fscanf(f,"epsh          : %lf",  &epsh          );
	  fscanf(f,"pretension    : %lf",  &pretension    );
	  fscanf(f,"maxForce      : %lf",  &maxForce      );
	  fscanf(f,"springMult1   : %lf",  &springMult1   );
	  fscanf(f,"delay         : %i",  &delay         );
	  fscanf(f,"splitBrain    : %i",  &splitBrain    );
	  return true;
}


////////////////// Begin controller   ///////////////////


void DerMartiusController::setInvDelayCoupling(bool invDC){
  if(invDC!=invDelayCoupling){
    invDelayCoupling=invDC;
    if(number_motors == number_sensors/2){ // Delay sensors
      Matrix M1(number_motors,number_sensors/2);
      M1.toId();
      M=M1.beside(M1*(invDelayCoupling? -1.0 : 1.0));
    }
  }
}


/***
 * @return motor target values.
 */
Matrix DerMartiusController::update(const Matrix& sensor_values, const Matrix& force_values){
  // Put new sensor vector into ring buffer

  forces      = (force_values + (-pretension))*(1/maxForce);
  x_raw_buffer[t] = sensor_values  + forces*springMult1;

  if(useDelay)
    xCurrent = x_raw_buffer[t].above(x_raw_buffer[t-delay]);
  else
    xCurrent = x_raw_buffer[t];

  x_buffer[t] = xCurrent; //input sensor values from robot

  int diff = 1;

  if (t > timedist + diff)
    {
      //        cout << "a.n " << M.getN() << "\n";
      //        cout << "b.m: " << x_buffer[t].getM() << "\n";
      Matrix mu = M * (x_buffer[t] - x_buffer[t-diff]);
      Matrix v =  x_buffer[t - timedist] -  x_buffer[t - timedist - diff];//default: timedist = 2 - 16
      // Learning step:
      double reg = pow(10.0f,-regularization);

      Matrix updateC =    mu * (v^T)* (1/(v.norm_sqr()+reg)); // normalized update

      if ( t > 10)
        {
          C_update += ((updateC   -  C_update)*urate).mapP(1,clip); //urate is  one over tau. default: urate=.02;
        }

      CC = C_update;

      ///////////////Begin neuron individual normalization:
      for (int i=0; i<number_motors; i++) {
        double normi = sqrt(CC.row(i).norm_sqr()); // norm of one row
        normmot.val(i,0) = .3/( normi + reg); // Georg disabled
      }
      CC = CC.multrowwise(normmot)*synboost;
      // synboost  corresponds to the gain factor kappa.
      ///////////////End neuron individual normalization
      if(splitBrain){ // C matrix is set to scaled unit matrix (ignore the learning above)
        CC=(CC^0)*synboost;
      }

      // Controller function
      // Matrix y =   (CC*x  + h).map(g);//Standard version. More recent:
      Matrix y =  (CC*x_buffer[t]  + h ).map(DerMartiusController::g);//Controller outputs the rate of change of motor values //default: maxspeed = .5
      //y = y.mapP(1.0,DerMartiusController::clip); //clipping of motor values

      //send y to buffer
      y_buffer[t] = y;
      yCurrent = y;

      // Bias learning
      if(epsh>=0)
        h += y * (-epsh) - h*.00001;
      else
        h*=0;

      //cout << "epsh:" << epsh <<endl;
      //(y^T).write(stdout);
      //(C).write(stdout);

      // limit force
      for(int i=0; i<y.getM(); i++){
        if(force_values.val(i,0)>maxForce){
          y.val(i,0)+=(force_values.val(i,0)-maxForce)/maxForce;
        }
      }

      guide(y);
      // send y to motors
      t++;
      stepNo=t;
      return y;
    }
  t++;
  stepNo=t;
  return y_buffer[0];
}

void DerMartiusController::guide(Matrix& y){
  switch (guideType){
  case 0:
    break;
  case 1: { // sine wave on the given channel
    int idx=guideIdx < 0 ? 0 : (guideIdx >= number_motors ? number_motors-1: guideIdx);
    y.val(idx,0) += sin(guideFreq*stepNo*2*3.141/100.0)*guideAmpl;
    break;
  }
  default:
    break;
  }
}


/* stores the controller values to a given file. */
bool DerMartiusController::store(FILE* f) const{
  // save matrix values
  C_update.store(f);
  h.store(f);
  M.store(f);
  storeParamsToFile(f);
  return true;
}

/* loads the controller values from a given file. */
bool DerMartiusController::restore(FILE* f){
  // save matrix values
  C_update.restore(f);
  h.restore(f);
  M.restore(f);
  restoreParamsFromFile(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}



bool DerMartiusController::storeToFile(const char* filename) const {
  FILE* f = fopen(filename,"wb");
  if(!f) return false;
  bool rv =store(f);
  fclose(f);
  return rv;
}

bool DerMartiusController::restoreFromFile(const char* filename) {
  FILE* f = fopen(filename,"rb");
  if(!f) return false;
  bool rv = restore(f);
  fclose(f);
  return rv;
}

///////////////// End of Controller ////////
