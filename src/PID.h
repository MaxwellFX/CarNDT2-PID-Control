#ifndef PID_H
#define PID_H

class PID {
private:
    // Error terms
    double p_error;
    double i_error;
    double d_error;

    double prevCTE;

    // Counters for twiddling
    long counter;   
	
    // Coefficient gains
    double tau_p;
    double tau_i;
    double tau_d;

    // Parameters for tuning
    unsigned int gainParamIndex;
    double tuningScale;
    double acceptingTotalCTE;
    double evaluationCount;
    double dp[2];
    double gain_param[2];
    int paramCntFailCount[2];
    bool doneAdding;
    bool doneSubstracting;

    bool IsConsecutiveFail();
    int getConsecutiveFailParamIndex();
    char operationType(int index);
public:

    double totalCTE;
    double bestError;

    // Counstructor
    PID();

    // Destructor
    virtual ~PID();

    // Initialize PID.
    void Init(double Kp, double Ki, double Kd);

    // Update the PID error variables given cross track error
    void UpdateError(double CTE);

    // Calculate the steering/throttling value based on the cross track error
    double GetSteeringValue();

    // set the tolerance. If the total error over the evaluating period is smaller than tolerance
    // then skip twiddling
    void SetTuningParams(int evaluationCount, double tolorance, double tuningScale);

    // Function for fine tuning
    void Twiddle(double CTE);
};

#endif /* PID_H */
