#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;
int valorMin_Max = 1900;
int valorMax_Max = 1350;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
if (pv > 315 && pv < 325) return 1519;
    // Calculate error
   double errortmp = setpoint;
    double error = setpoint - pv;
    std::cout << "pv: " << pv <<std::endl;

    std::cout << "Erro: " << error <<std::endl;

    // Proportional term
    double Pout = _Kp * error;
   //double Pout = _Kp * errortmp;
    std::cout << "Pout: " << Pout <<std::endl;

    // Integral term
    _integral += error * _dt;
    //_integral = errortmp * _dt;
    if(_integral > _max ) _integral = _max;
    else if(_integral < _min) _integral = _min;
    double Iout = _Ki * _integral;

    std::cout << "Iout: " << Iout <<std::endl;


    // Derivative term
    double derivative = (/*error*/ error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    std::cout << "Dout: " << Dout <<std::endl;


    // Calculate total output
    double output = Pout + Iout + Dout + 550;

    std::cout << "output: " << output <<std::endl;

int aux;
int valorMin_actual = output;

if(output > valorMax_Max && output < 1900)
        valorMax_Max = output;

if(valorMin_actual < valorMin_Max && output > 1350)
	valorMin_Max = valorMin_actual;
 std::cout << "MAX: " << valorMax_Max <<std::endl;
 std::cout << "MIN: " << valorMin_Max <<std::endl;

int aux0 = valorMax_Max - valorMin_Max;
 std::cout << "aux0: " << aux0 <<std::endl;

aux = output - valorMin_Max;
int aux2;
if(aux0 != 0)
aux2 = (100 * aux) / aux0;
else
aux2 = (100 * aux) / 300;

 std::cout << "aux: " << aux <<std::endl;
 std::cout << "aux2: " << aux2 << "%" <<std::endl;

    // Restrict to max/min
 //   if( output > _max )
int aux3;
aux3 = _max - _min;
output = ( (aux2 * aux3) / 100) + _min;
std::cout << "output2: " << output <<std::endl; 


	if(aux2 > 70)
   	output = _max;
//    else if( output < _min )
	 else if(aux2 < 30)
	  output = _min;

 std::cout << "output3: " << output <<std::endl;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
