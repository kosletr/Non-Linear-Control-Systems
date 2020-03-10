# Non-Linear-Control-Systems

## Description

This repository contains code written by me, Konstantinos Letros, as a project for Non Linear Control Systems (Control Systems III) course during the 7th semester (academic year: 2018-2019) of my undergraduate studies in Electrical Engineering (Section: Electronics and Computer Science) at Aristotle University of Thessaloniki. 

## Implementations

### Part A - Behavioral Analysis of a Non Linear System

This part contains the behavioral analysis of a system including the non linear function M shown bellow.

Block Diagram              |  Non Linear Function m(e)
:-------------------------:|:-------------------------:
![](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/1b.jpg)  |  ![](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/m_func.jpg)

Behavioral Anaylisis is achivied by using different inputs r, initial conditions x(0) and by plotting different enlightening graphs.

![Response Example](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/resp13.jpg)

### Part B - Control of a Non Linear Robotic System

This part contains the construction and derivation of a suitable non linear controller as well as the corresponding non linear control analysis to prove system's stability. Moreover, in order to approach a more realistic scenario, all of the parameter-values taken place are not exactly known, but have uncertainties.

![Robotic_System](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/robot.png)

Furthermore, apart from apachieving stability, the controller should also be able to follow a certain sinusodial trajectory after a very short transitional period, as shown bellow.

Controller                 |  Sinusodial Trajectory
:-------------------------:|:-------------------------:
![](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/Acon2.jpg)  |  ![](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/Ap3.jpg)

Finaly, a parameter estimation procedure takes place in order to estimate the currently uknown - exact values of the uncertain parameters.

b Parameter Estimation     |  h Parameter Estimation 
:-------------------------:|:-------------------------:
![](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/b_est_1.jpg)  |  ![](https://github.com/kosletr/Non-Linear-Control-Systems/blob/master/Latex/h_est_1.jpg)

## Applications used for this implementation:
 - MathWorks MATLAB R2018b
 - MiKTeX
 - Texmaker
