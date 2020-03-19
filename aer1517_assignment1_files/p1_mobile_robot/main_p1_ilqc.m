% main_p1_ilqc: Main script for Problem 1.1 ILQC controller design.
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

%% General
% add subdirectories
addpath(genpath(pwd));

% add task
task_ilqc = task_design();
N = length(task_ilqc.start_time:task_ilqc.dt:task_ilqc.end_time);

% add model
const_vel = 1; % assume constant forward speed
model = generate_model(const_vel);

% save directory
save_dir = './results/';

% initialize controller
load(strcat(save_dir, 'lqr_controller'));
controller_ilqc = controller_lqr;

% flags
plot_on = true;
save_on = true;

%% [Problem 1.1 (j)] Iterative Linear Quadratic Controller
% =========================== [TODO] ILQC Design ==========================
% Design an ILQC controller based on the linearized dynamics and
% quadratized costs. The cost function of the problem is specified in
% 'task_ilqc.cost' via the method 'task_design()'.
%
%
% =========================================================================

Q_s = task_ilqc.cost.params.Q_s;
R_s = task_ilqc.cost.params.R_s;
Q_t = task_ilqc.cost.params.Q_t;

controller_ilqc_last = zeros(size(controller_ilqc));
iter = 0;

% task_ilqc.start_x = [0, pi]';

while norm(controller_ilqc - controller_ilqc_last, 2) > 10^-5  && iter < task_ilqc.max_iteration
    % Something
    controller_ilqc_last = controller_ilqc;
    iter = iter+1;

    % Init policy as controller_ilqc(:, k) = 0 for all k
    lin_out = mobile_robot_sim(model, task_ilqc, controller_ilqc);
    x_lin = lin_out.x;
    u_lin = lin_out.u;

    [q_N, q_v_N, Q_N] = terminal_cost_quad(Q_t, task_ilqc.goal_x, x_lin(:, end));
    s_kp = q_N; s_v_kp = q_v_N; S_kp = Q_N;

    for k = N-1:-1:1
        [A_k, B_k] = mobile_robot_lin(x_lin(:, k), u_lin(:, k), ...
                                              task_ilqc.dt, const_vel);
        [ q_k, q_v_k, Q_k, r_k, R_k, P_k ] = ...
                         stage_cost_quad( Q_s, R_s, ...
                                          task_ilqc.goal_x, task_ilqc.dt, ...
                                          x_lin(:, k), u_lin(:, k) );
        [U_ff, U_fb, s_k, s_v_k, S_k] = update_policy(A_k, B_k, ...
                                                     q_k, q_v_k, Q_k, ...
                                                     r_k, R_k, P_k, ...
                                                     s_kp, s_v_kp, S_kp, ...
                                                     x_lin(:, k), u_lin(:, k));
        controller_ilqc(:, k) = [U_ff; U_fb];
        S_kp   = S_k;
        s_v_kp = s_v_k;
        s_kp   = s_k;
    end
end


%% Simulation
sim_out_ilqc = mobile_robot_sim(model, task_ilqc, controller_ilqc);
fprintf('\n\ntarget state [%.3f; %.3f]\n', task_ilqc.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_ilqc.x(:,end));

%% Plots
if plot_on
    plot_results(sim_out_ilqc);
end


%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'ilqc_controller'), 'controller_ilqc', ...
        'sim_out_ilqc', 'task_ilqc');
end


%% Helper Functions

function [x_kp] = mobile_robot_kin(x_k, u_k, dt, v_bar)
   
    if dt <= 0
       disp('mobile_robot_kin: Invalid dt')
       return
    end

    x_k = reshape(x_k, numel(x_k), []);
    u_k = reshape(u_k, numel(u_k), []);
    
    x_kp = x_k + dt * [ v_bar * sin(x_k(2)); u_k(1) ];
end

function [A_k, B_k] = mobile_robot_lin(x_lin, u_lin, dt, v_bar)
        
    add_error = false;

    A_k = [];
    B_k = [];
    if dt <= 0
       disp('mobile_robot_lin: Invalid dt')
       return 
    end
    A_k = [ 1, dt*v_bar*cos(x_lin(2));
            0 , 1];
    B_k = [ 0; dt];
    
    if add_error
        A_k = A_k * (1 + 0.2);
        B_k = B_k * (1 + 0.2);
    end
end

function [q_N, q_v_N, Q_N] = terminal_cost_quad(Q_t, x_goal, x_lin)
    
    x = x_lin - x_goal;
    x = reshape(x, numel(x), []);
    
    q_N   = 1/2 * x' * Q_t * x;
    q_v_N = 1/2 * (Q_t' + Q_t) * x;
    Q_N   = 1/2 * (Q_t' + Q_t);
end

function [ q_k, q_v_k, Q_k, r_k, R_k, P_k ] = stage_cost_quad(Q_s, R_s, x_goal, dt, x_lin, u_lin)
        
    if dt <= 0
       disp('stage_cost_quad: Invalid dt')
       return
    end

    x = x_lin - x_goal;
    x = reshape(x, numel(x), []);
    u = u_lin;
    u = reshape(u, numel(u), []);
    
    q_k   = 1/2 * x' * Q_s * x + 1/2 * u' + R_s * u * dt;
    q_v_k = 1/2 * (Q_s' + Q_s) * x * dt;
    Q_k   = 1/2 * (Q_s' + Q_s) * dt;
    
    r_k = 1/2 * (R_s' + R_s) * u * dt;
    R_k = 1/2 * (R_s' + R_s) * dt;
    
    P_k = zeros(numel(u), numel(x)) * dt;
    
end

function [U_ff, U_fb, s_k, s_v_k, S_k] = update_policy(A_k, B_k, ...
                                                   q_k, q_v_k, Q_k, ...
                                                   r_k, R_k, P_k, ...
                                                   s_kp, s_v_kp, S_kp, ...
                                                   x_lin, u_lin)
    
    x_lin = reshape(x_lin, numel(x_lin), []);
    u_lin = reshape(u_lin, numel(u_lin), []);

    g_k = r_k + B_k' * s_v_kp;
    G_k = P_k + B_k' * S_kp * A_k;
    H_k = R_k + B_k' * S_kp * B_k;
    
    H_k = 1/2*(H_k + H_k');

    u_ff = -inv(H_k) * (g_k);
    K_k  = -inv(H_k) * (G_k);
    
    U_ff = u_lin + u_ff - K_k * x_lin;
    U_fb = K_k';
    
    S_k   = Q_k + A_k' * S_kp * A_k + K_k' * H_k * K_k + ...    
                                         K_k' * G_k + G_k' * K_k; 
    s_v_k = q_v_k + A_k' * s_v_kp + K_k' * H_k * u_ff + ...    
                                         K_k' * g_k + G_k' * u_ff;
    s_k   = q_k + s_kp + 1/2 * u_ff' * H_k * u_ff + u_ff' * g_k;
    
end                                                   