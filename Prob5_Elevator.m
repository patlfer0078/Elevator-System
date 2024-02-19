% Fernando Patlan, MECE 6341 Modeling of Physical Systems
% Problem 5 - Electromechanical System - Elevator 
% Simulink Relative Tolerance was lowered to 1e-5

% clear all previous command codes and Variables
clc
clear all

% Constants
Jm = 4.2e-6; % mass moment of inertia
K = 100000; % spring stiffness
ba = 2.6e-4; % damping coefficient
ei = 15; % input voltage
g = 9.81; % gravity
La = 0.001; % Inductance
Ra = 1.5; % Resistor
r1 = 0.03; % radius, gear 1
r2 = 225*r1; % radius, gear 2
r3 = 0.05; % radius, shaft for elevator
km = 0.0232; % torque constants
kb = 0.0232; % back efm constant
m = 18; % elevator mass
b = 2000*ba; % damping of elevator/wall N/(m/s)


% State of System
A = [0 1 0 0 0;
    -K/(m*r3^2) -b/m (K*r1)/(m*r2*r3) 0 0;
    0 0 0 1 0;
    (K*r1)/(Jm*r2*r3) 0 (-K*r1^2)/(Jm*r2^2) -ba/Jm km/Jm;
    0 0 0 -km/La -Ra/La];
B = [0 0;
     0 -1;
     0 0;
     0 0;
     1/La 0];
C = [0 1 0 0 0;
     0 0 0 1 0];
D = [0 0;
     0 0];
x0 = [0 0 0 0 0]'; %initial conditions
tf = 1;  %final time in seconds
t2 = [0:0.001:tf];  %time interval in steps of 0.1 seconds

% Simulink call
simdata = sim('Prob5_Simulink');
elevator_vel = simdata.y_out(:,1);
motor_vel = simdata.y_out(:,2);
t1 = simdata.t1;
%Plotting outputs
figure(1)
%Position of gear 2
subplot(211)
plot(t1, elevator_vel, '-b', LineWidth=1.5)
xlabel('Time (Sec)', 'fontsize', 10)
ylabel('Velocity (m/s)', 'fontsize', 10)
grid on
%Velocity of gear 2
subplot(212)
plot(t1, motor_vel, '-b', LineWidth=1.5)
xlabel('Time (Sec)', 'fontsize', 10)
ylabel('Angular Vel. (rad/s)', 'fontsize', 10)
grid on
