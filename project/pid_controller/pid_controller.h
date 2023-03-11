/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

    double Error_p;
    double Error_i;
    double Error_d;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * Output limits
    */
    double outputMax;
    double outputMin;
  
    /*
    * Delta time
    */
    double deltaTime;
  
  
   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
   double _p_err;
   double _i_err;
   double _d_err;
  
     
  
    /*
    * Coefficients
    */
   double _kp;
   double _ki;
   double _kd;
   
  
    /*
    * Output limits
    */
   double _output_lim_max;
   double _output_lim_min;  
  
  
    /*
    * Delta time
    */
   double _delta_time;
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


