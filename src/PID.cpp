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

    this->gainParamIndex = 2;

    this->consecutiveUnderPerformIndexArray[0] = 0;
    this->consecutiveUnderPerformIndexArray[1] = 0;
    this->consecutiveUnderPerformIndexArray[2] = 0;

    this->paramsUpdateFlag[0] = false;
    this->paramsUpdateFlag[1] = false;
    this->paramsUpdateFlag[2] = false;

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

    if(this->counter % this->evaluationCount == 0) 
    {
        cout<<"update count: "<< this->counter / this->evaluationCount << endl;
        cout<<"total Error: "<< this->totalCTE << endl;
        cout<<"best Error: "<< this->bestError << endl;
        cout<<"accepting tolerance: " << this->acceptingTotalCTE << endl;
        
        if (this->bestError <= this->acceptingTotalCTE) {
            cout<< "accepting tolerance has been met, no need for updating" << endl;
        } else {
            if(this->totalCTE < this->bestError) 
            {
                this->bestError = this->totalCTE;
                cout<<"new best error: " << this-> bestError << endl;

                // Ingore the first encounter
                if (this->counter != this->evaluationCount) {
                    this->dp[this->gainParamIndex] *= (1.0 + this->tuningScale);
                    this->paramsUpdateFlag[this->gainParamIndex] = true;
                }
                
                this->doneAdding = false;
                this->doneSubstracting = false;

            } else {
                cout << "Underperform 1st time for ["<<this->gainParamIndex<<"]"<<endl;
                this->consecutiveUnderPerformIndexArray[this->gainParamIndex] += 1;
            }

            int consecutiveUnderPerformingIdx =  this->GetConsecutiveFailingParamIndex();
            
            if(!this->doneAdding && !this->doneSubstracting) {
                this->TurnGainParam(this->gainParamIndex, this->dp[this->gainParamIndex]);
                this->doneAdding = true;
            } else if (this->doneAdding && !this->doneSubstracting) {
                this->TurnGainParam(this->gainParamIndex, -2 * this->dp[this->gainParamIndex]);
                this->doneSubstracting = true;
            } else if (consecutiveUnderPerformingIdx != -1 && consecutiveUnderPerformingIdx <= 2) {
                cout << "update failed, fall back" <<endl;
                this->dp[consecutiveUnderPerformingIdx] *= (1.0 - this->tuningScale);
                this->paramsUpdateFlag[this->gainParamIndex] = true;
                this->TurnGainParam(consecutiveUnderPerformingIdx, this->dp[consecutiveUnderPerformingIdx]);
            }

            this->totalCTE = 0;
            if (this->counter >= 100 * this->evaluationCount) {
                this->counter = 0;
            }
            if (this->paramsUpdateFlag[this->gainParamIndex]) {
                this->paramsUpdateFlag[this->gainParamIndex] = false;
                this->gainParamIndex = (this->gainParamIndex + 1) % 3;
            }
            cout<< "new parameters: " << endl;
            cout<< "p: "<< this->tau_p << ", i: " << this->tau_i << ", d: " << this->tau_d << endl;
        }
    }

}

void PID::TurnGainParam(int paramIndex, double delta) {
    if (paramIndex == 0) {
        cout<< "Adding (" << delta << ") to Porpotional gain" << endl;
        this->tau_p += delta;
    } else if (paramIndex == 1) {
        cout<< "Adding (" << delta << ") to Differential gain" << endl;
        this->tau_d += delta;
    } else if (paramIndex == 2) {
        cout<< "Adding (" << delta << ") to Integral gain" << endl;
        this->tau_i += delta;
    } else {
        throw "Invalid parameter index!";
    }
}

int PID::GetConsecutiveFailingParamIndex() {
    for (unsigned int i = 0; i < sizeof(this->consecutiveUnderPerformIndexArray); i++ ) {
        if (this -> consecutiveUnderPerformIndexArray[i] >= 2) {
            return i;
        }
    }
    return -1;
}