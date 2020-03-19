% plot_results: Function for plotting simulation results
%
% Input:
%       sim_out: A structure that contains the continuous time t, states x,
%       inputs u at each time step.
%
% --
% Control for Robotics
% AER1517 Spring 2020
% Programming Exercise 1
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
%
% --
% Revision history
% [20.01.31, SZ]    first version

function [ plots ] = plot_results(sim_out)
    figure(1);
    
    subplot(3,1,1);
    plots(1) = plot(sim_out.t, sim_out.x(1,:));
    ylabel('y [m]');
    hold on;
    
    subplot(3,1,2);
    plots(2) = plot(sim_out.t, sim_out.x(2,:));
    xlabel('t [sec]');
    ylabel('theta [rad]');
    hold on;
    
    subplot(3,1,3);
    plots(3) = plot(sim_out.t(1:end-1), sim_out.u(1,:));
    xlabel('t [sec]');
    ylabel('omega [rad/sec]');
end

