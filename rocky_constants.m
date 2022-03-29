% rocky
% disturbance rejection
% critically damped

clear all close all 

l = 0.4;
g = 9.8;


% motor
kmotor = .63;
tau = .0325;

%% rocky11, poles at (-5, 0), (-5, +-25deg), (-5, +-35deg)
kp = 5818.9
ki = 29095
jp = 950.15
ji = -4758.2
ci = -5345.2

%% Forward movement test (adding steady-state error back in)
kp = 5818.9
ki = 29095
jp = 950.15
ji = 0
ci = 0