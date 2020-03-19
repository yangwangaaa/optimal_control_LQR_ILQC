% main_p2_ilqc: Main script for Problem 1.2 ILQC controller design.
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
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.01.31, SZ]    first version

close all;
clear all;
clc;

%% General
% add subdirectories
addpath(genpath(pwd));

% define task
task = Task_Design();

% load the dynamic model of the quadcopter
load('Quadrotor_Model.mat', 'Model'); % save as structure "Model" 

% define cost function
% [Problem 1.2 (e)] Fill in the missing parts under 'via_point' in ...
% Cost_Design( Model.param.mQ, task )
task.cost = Cost_Design( Model.param.mQ, task );

% save directory
save_dir = './Results/';

% flags
plot_on = true;
save_on = true;
load_lqr = false;

%% Initial Controller (from Previous Part)
if load_lqr
    % load saved LQR controller
    load(strcat(save_dir, 'lqr_controller'));
else 
    % LQR controller design
    [Initial_Controller, Cost_LQR] = LQR_Design(Model, task);

    % simulation
    sim_out_lqr = Quad_Simulator(Model,task,Initial_Controller);
    
    disp('LQR controller performance:');
    fprintf('Cost with LQR controller (metric: LQR cost function!): J* = %.3f \n', Cost_LQR);
    fprintf('Start Quadcopter position: x = %.3f, y = %.3f, z = %.3f \n', sim_out_lqr.x(1:3,1));
    fprintf('Final Quadcopter position: x = %.3f, y = %.3f, z = %.3f \n\n', sim_out_lqr.x(1:3,end));
end


%% Problem 2: ILQC controller design 
% [Problem 1.2 (d)] Fill in the missing parts in ...
% ILQC_Design(Model,task,Initial_Controller,@Quad_Simulator)

[ILQC_Controller, Cost] = ILQC_Design(Model,task,Initial_Controller,@Quad_Simulator);

%% Simulation
t_cpu = cputime;
Sim_Out_ILQC = Quad_Simulator(Model,task,ILQC_Controller);
t_cpu = cputime - t_cpu;
fprintf('The ILQC algorithm found a solution in %fs \n\n',t_cpu);
fprintf('Final Quadcopter Position: xQ = %.3f, yQ = %.3f, zQ = %.3f \n', Sim_Out_ILQC.x(1:3,end));
fprintf('Final Quadcopter Velocity: xQ = %.3f, yQ = %.3f, zQ = %.3f \n', Sim_Out_ILQC.x(7:9,end));

%% Visualization of ILQC controller
if plot_on
    Visualize2(Sim_Out_ILQC, Model.param);
end

%% Save Results and Figures
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'ilqc_controller'), 'ILQC_Controller', 'Sim_Out_ILQC', 'task');
    
	% save visualization plot for report
    if plot_on
        saveas(gcf, strcat(save_dir, 'ilqc_controller'), 'png');
    end
end