%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
load('stepdata.mat');
%% Define motor parameters
K = 1.7; % DC gain [rad/Vs]
Kp = 5;
Ki = 0.75;
sigma = 11; % time constant reciprocal [1/s]
theta_d = pi;
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('Position_Controller')
%
% run the simulation
%
out=sim('Position_Controller');
%% A Plot of the results
%
figure
subplot(2,1,1)
plot(out.Current_pos,'linewidth',2)
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(2,1,2)
plot(out.Voltage,'linewidth',2)
xlabel('Time (s)')
ylabel('Voltage (V)')