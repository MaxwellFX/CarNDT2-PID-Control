#include "PID.h"
#include <limits>
#include <cmath>
#include <iterator>
#include <iostream>

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
    this->dp[0] = 0.1 * Kp;
    this->dp[1] = 0.1 * Kd;
    this->dp[2] = 0.1 * Ki;

    this->gainParamIndex = 0;

    this->consecutiveUnderPerformIndexArray[0] = 0;
    this->consecutiveUnderPerformIndexArray[1] = 0;
    this->consecutiveUnderPerformIndexArray[2] = 0;

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
    return -this->tau_p * this->p_error 
            - this->tau_i * this->i_error 
            - this->tau_d * this->d_error;
}

void PID::SetTuningParams(int evaluationCount, double tolerance, double tuningScale = 0.05) 
{
    this->tuningScale = tuningScale;
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
        cout<<"total Error: "<< this->totalCTE << endl;
        cout<<"best Error: "<< this->bestError << endl;

        if(this->totalCTE < this->bestError) 
        {
            this->bestError = this->totalCTE;
            cout<<"new best error: " << this-> bestError << endl;

            this->dp[this->gainParamIndex] *= (1.0 + this->tuningScale);
            this->doneAdding = false;
            this->doneSubstracting = false;
        }

        if(!this->doneAdding && !this->doneSubstracting) {
            this->TurnGainParam(this->gainParamIndex, this->dp[this->gainParamIndex]);
            this->doneAdding = true;
        } else if (this->doneAdding && !this->doneSubstracting) {
            this->TurnGainParam(this->gainParamIndex, -this->dp[this->gainParamIndex]);
            this->doneSubstracting = false;
        } else {
            this->dp[this->gainParamIndex] *= (1.0 - this->tuningScale);
        }

        this->gainParamIndex = (this->gainParamIndex + 1) % 3;
        this->totalCTE = 0;
        this->counter = 0;
        cout<< "new parameters: " << endl;
        cout<< "p: "<< this->tau_p << ", i: " << this->tau_i << ", d: " << this->tau_d << endl;
    }

}

void PID::TurnGainParam(int paramIndex, double delta) {
    if (paramIndex == 0) {
        cout<< "Adding (" << delta << ") to P" << endl;
        this->tau_p += delta;
    } else if (paramIndex == 1) {
        cout<< "Adding (" << delta << ") to D" << endl;
        this->tau_d += delta;
    } else if (paramIndex == 2) {
        cout<< "Adding (" << delta << ") to I" << endl;
        this->tau_i += delta;
    } else {
        throw "Invalid parameter index!";
    }
}

int PID::GetConsecutiveFailingParamIndex() {
    return -1;
}