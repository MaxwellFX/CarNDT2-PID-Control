# **Self-Driving Car Engineer Nanodegree Program - PID Control Project**

## **Introduction**

PID controller stands for Proportional-Integral-Derivative Controller. A PID controller continuously calculates the error value as the difference between a desired coordinates and a measured process variable and applies a control signal based on proportional, integration and differential terms. 

In this project, we will utilize PID controller to maneuver the self-driving car around the track. The simulator will provide Cross-Track-Error (CTE) and speed for us to compute the proproper steering and throttling value to keep the car reliably driving on the track. 

---
## **Rubric Discussion Points**

### **Cross Track Error :**

In this project, a cross track error is defined as the distance from the vehicle to the target trajectory. 

### **Proportional component :**

The proportional control steers towards the target trajectory harder the further away from the trajectory, hence the "proportional". The component is defined by:
    
    P = -tau_p * cte

where `tau_p` is the proportional gain and `cte` is the distance between the vehicle and the desired trajectory. However, proportional controller often overshoot around the trajectory, results in the car turning around the target line but never reaches it, thus, additional components are needed.

### **Differential component :**

The differential components measures how fast the vehicle is moving towards/away from the trajectory, as a counter measure to soften the overshooting behaviour of the proportional component. It is defined by:

    D = -tau_d * (current_cte - previous_cte)

where `tau_d` is the differential gain for the rate of change `(current_cte - previous_cte)`. As a result Differential controller will prevent vehicle steer towards the track too quick, hence compensating the overshooting. Unfortunately, this is still not enough, as unexpected factors, such as external environment or mechanical failure, could add distance bias to the vehicle. This makes the car steering onto a trajectory offset without ever reaching to the trajectory itself. Therefore, we need another component to correct this.

### **Integral component :**

The Integral component sums up the CTE to give a indication if the vehicle is spending more time on one side of the trajectory or the other. It is defined by: 

    I = - tau_i * sum(cte)

where `tau_i` is the gain for how strong we want to counter against the integral. 

Finally, with all three components, a PID controller is made. 

### **Visual demonstration:**

**Note that for the purpose of comparison, throttle input value has been kept as constant for the following plot.** Here is a visual comparison on how each components contributes to the vehicle control:

<p align="center"><a href="https://www.youtube.com/watch?v=wy60VDzF2-8"> Video demo for removing I, D components </a><p align="center">
<p align="center"><a href="https://www.youtube.com/watch?v=FHBf_rd32X0"> Video demo for removing I component </a><p align="center">
<p align="center"><a href="https://www.youtube.com/watch?v=-5a1QE-rMoA"> Video demo for removing D component </a><p align="center">

<p align="center"><img src = "./pythonGraph/CTE.PNG" alt = "CTE comparison for differen controllers" width = "400px" class ="center"><p align="center">

As we can see from the graph, without the compensation from the differential component, the run failed disastrously. Both P and PI controller eventually wonders off from the track. Here is another graph showing the corresponding steering value:

<p align="center"><img src = "./pythonGraph/Steering angle.PNG" alt = "CTE comparison for differen controllers" width = "400px" class ="center"><p align="center">

As expected, the steering value indicates for PD and PID controller, the vehicle was moving relatively along the desired trajectory with few instances of anomaly. 

Between PD and PID controller, we can observe that the integral components reduces the CTE slightly comparing to PD only.

### **Hyperparameters:**

In addition to controlling the steering angle to PID controller, I have also added another controller for the throttling value. This additional throttling controller makes the automated twiddling very time consuming since the car slows down or moving back and forth when it detects large increases of CTE. 

Thus, I mannually tuned the hyperparameters through rough guess until the vehicle can somewhat follows the track. Then I start the fine tuning processing via the twiddling function introduced from the class lecture. Each evaluation process is mearsured over 2150 steps, which is roughly how long it takes to complete one track. After 10 laps or so, I ended up with the parameter (P:0.1744, I:0.00033837, D:2.525)


<a href="https://www.youtube.com/watch?v=0nKvZvAKHvk">Here is the video demo with the final parameter</a>