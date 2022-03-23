% rocky
% disturbance rejection
% critically damped
%
clear all close all 

l = 0.4;
g = 9.8;


% motor
kmotor = .92;
tau = .0511;


pole = 10;
kp = pole*l*10;
ki = (kp^2/l + 4*g)/4;

% kp = 2.2104e+03
% ki = 1.9509e+03

ji = 1
jp = 1

ci = 50


% kp = 20
% ki = 100
