Distributionally Robust Risk Map for Learning-Based Motion Planning and Control: A Semidefinite Programming Approach
====================================================

This repository includes an official python implementation of **DR-RRT*** and **DR-MPC** presented in **[Distributionally Robust Risk Map for Learning-Based Motion
Planning and Control: A Semidefinite Programming Approach](https://arxiv.org/pdf/2105.00657.pdf)**


## 1. Requirements
- All our experiments are performed in **[MATLAB](https://kr.mathworks.com/products/matlab.html)**
- The DR-risk map is constructed by solving the SDP problem using **[MOSEK](https://www.mosek.com)**
- The MPC is solved using **[FORCESPRO 6.0.0](https://forces.embotech.com/)**

To install FORCESPRO, you first need to get license. If you are currently working as a researcher you can request academic license. For detailed installation guide refer to [official manual](https://forces.embotech.com/Documentation/index.html).

## 2. Quick Start

### 2.1. DR-RRT*

To test the performance of our motion planning algorithm, we have implemented to environments:
- Highway Scenario
- Intersection Scenario

To run the experiments, first navigate either to `DR-Risk-Map/DR-RRTStar/Highway` or to `DR-Risk-Map/DR-RRTStar/Intersection` then simply run the script `DR_RRTStar.m`.

Example:
```
cd DR-Risk-Map/DR-RRTStar/Highway
DR_RRTStar
```

### 2.2. DR-MPC

For DR-MPC Neural Networks are trained to construct the risk maps for several Wasserstein radii. The corresponding weights are saved in `DR-Risk-Map/DR-MPC/NN_Weights/`.

To run the experiment for the service robot, navigate to `DR_Risk-Map/DR-MPC` and run the script `ServiceRobot.m`. It will first generate a FORCESPRO solver scripted in `ServiceRobot_Forces_Gen.m` then apply the controller to the robot.

Example:
```
cd DR-Risk-Map/DR-MPC
ServiceRobot
```


