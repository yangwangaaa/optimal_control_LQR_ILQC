% main_p1_lqr: Main script for Problem 1.1 LQR controller design.
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

clear all;
close all;
clc;

%% General
% add subdirectories
addpath(genpath(pwd));

% define task
task_lqr = task_design();
N = length(task_lqr.start_time:task_lqr.dt:task_lqr.end_time);

% add model
const_vel = 1; % desired forward speed
model = generate_model(const_vel);

% initialize controller
controller_lqr = zeros(3, N-1);

% save directory
save_dir = './results/';

% flags
plot_on = true;
save_on = true;

%% [Problem 1.1 (c)] LQR Controller
% =========================== [TODO] LQR Design ===========================
% Design an LQR controller based on the linearization of the system about
% an equilibrium point (x_eq, u_eq). The cost function of the problem is
% specified in 'task_lqr.cost' via the method 'task_design()'.
%
% 
% =========================================================================

% task_lqr.start_x = [0, pi]';

ratio = [0.1, 1, 10, 100];
% Mode 0 is Default
% Mode 1 is compare R vs Q_y
% Mode 2 is compare Q_h vs Q_y
% Mode 3 is Check effects of X_0
mode = 0;
Q = task_lqr.cost.params.Q_s;
R = task_lqr.cost.params.R_s;

if mode == 0
    ratio = [1];
end

for i = 1:length(ratio)
    y_target = 1;
    A = [ 0, const_vel;
          0, 0];
    B = [ 0;
          1];
    
    if mode == 0
        % Do nothing
    elseif mode == 1
        R = ratio(i)*Q(1,1);
    elseif mode == 2
        Q = diag([1,ratio(i)]);
        R = [1];
    elseif mode == 3
        task_lqr.start_x(1) = 3*(1 + log10(ratio(i)));
    end
    
    [K, S, E] = lqr(A, B, Q, R);

    theta_ff = 0 + K*task_lqr.goal_x;
    theta_fb = -K';

    for j = 1:N-1
        controller_lqr(1:3, j) = [theta_ff; theta_fb];
    end

    %% Simulation
    sim_out_lqr = mobile_robot_sim(model, task_lqr, controller_lqr);
    fprintf('--- LQR ---\n\n');
    fprintf('trajectory cost: %.2f\n', sim_out_lqr.cost);
    fprintf('target state [%.3f; %.3f]\n', task_lqr.goal_x);
    fprintf('reached state [%.3f; %.3f]\n', sim_out_lqr.x(:,end));

    %% Plots
    if plot_on
        plot_results(sim_out_lqr); hold on;
    end
end
if mode == 1
    legend( 'R_s = 0.1Q_{y}', 'R_s = Q_{y}', 'R_s = 10Q_{y}', 'R_s = 100Q_{y}' )
elseif mode == 2
    legend( 'Q_{h} = 0.1Q_{y}', 'Q_{h} = Q_{y}', 'Q_{h} = 10Q_{y}', 'Q_{h} = 100Q_{y}' )
elseif mode == 3
    legend( "y_{start} = " + int2str(3*(1 + log10(ratio(1)))) + " m",...
            "y_{start} = " + int2str(3*(1 + log10(ratio(2)))) + " m",...
            "y_{start} = " + int2str(3*(1 + log10(ratio(3)))) + " m",...
            "y_{start} = " + int2str(3*(1 + log10(ratio(4)))) + " m" );
end
hold off;

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'lqr_controller'), 'controller_lqr', ...
        'sim_out_lqr', 'task_lqr'); 
end