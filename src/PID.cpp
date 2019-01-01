#include "PID.h"
#include <limits>
#include <cmath>
#include <iterator>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
	this->tau_p = Kp;
	this->tau_d = Kd;
	this->tau_i = Ki;
	
	this->p_error = 0.0;
	this->i_error = 0.0;
	this->d_error = 0.0;
	
	// Previous cte.
    this->prevCTE = 0.0;

    // Counters.
    this->counter = 0;
    this->acceptingTotalCTE = 0.0;
    this->bestError = numeric_limits<double>::max();

    // twiddle gain
    this->dp[0] = 0.1 * tau_p;
    this->dp[1] = 0.1 * tau_d;
    this->dp[2] = 0.1 * tau_i;

    this->gain_param[0] = tau_p;
    this->gain_param[1] = tau_d;
    this->gain_param[2] = tau_i;

    this->doneAdding = false;
    this->doneSubstracting = false;
}

void PID::UpdateError(double CTE)
{
	// Proportional error.
    this->p_error = CTE;

    // Integral error.
    this->i_error += CTE;

    // Diferential error.
    this->d_error = CTE - this->prevCTE;
	
    this->prevCTE = CTE;
}

double PID::GetSteeringValue()
{
	// return -this->tau_p * this->p_error - this->tau_i * this->i_error - this->tau_d * this->d_error;
    return -this->gain_param[0] * this->p_error 
            - this->gain_param[2] * this->i_error 
            - this->gain_param[1] * this->d_error;
}

void PID::SetTuningParams(int evaluationCount, double tolerance, double tuningScale = 0.05) 
{
    this->tuningScale = 1 + tuningScale;
    this->evaluationCount = evaluationCount;
    this->acceptingTotalCTE = tolerance;
}

void PID::Twiddle(double CTE) 
{
    // Proportional error.
    this->p_error = CTE;

    // Integral error.
    this->i_error += CTE;

    // Diferential error.
    this->d_error = CTE - this->prevCTE;
	
    this->prevCTE = CTE;

    // Used for tuning evaluation
    this->totalCTE += fabs(CTE);
    this->counter++;

    if(this->counter == this->evaluationCount) 
    {
        if(this->totalCTE < this->bestError) 
        {
            this->bestError = this->totalCTE;
            this->dp[this->gainParamIndex] *= this->tuningScale;
            // this->gainParamIndex = (this->gainParamIndex + 1) % 3;
            this->doneAdding = false;
            this->doneSubstracting = false;
        } 
        if(!this->doneAdding && !this->doneSubstracting) {
            this->gain_param[this->gainParamIndex] += this->dp[this->gainParamIndex];
            this->doneAdding = true;
        } else if (this->doneAdding && !this->doneSubstracting) {
            this->gain_param[this->gainParamIndex] -= 2*this->dp[this->gainParamIndex];
            this->doneSubstracting = false;
        } else {
            this->gain_param[this->gainParamIndex] -= 2*this->dp[this->gainParamIndex];
        }

        this->totalCTE = 0;
        this->counter = 0;
    }
}