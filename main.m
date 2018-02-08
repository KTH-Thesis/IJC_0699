function main

  %% Initialization
  clc;
  clear all;
  close all;
  tic;

  %% Global parameters

  num_states = 3;
  num_inputs = 2;

  % Agent 1
  global t_1;
  global x_1;
  global u_1;
  global des_1;
  global u_open_loop_1;
  global x_open_loop_1;

  % Agent 2
  global t_2;
  global x_2;
  global u_2;
  global des_2;
  global u_open_loop_2;
  global x_open_loop_2;

  % Agent 3
  global t_3;
  global x_3;
  global u_3;
  global des_3;
  global u_open_loop_3;
  global x_open_loop_3;

  % Penalty matrices
  global Q;
  global R;
  global P;

  % Input bounds
  global u_max;
  global u_min;

  % Obstacles
  global obs;
  global r;

  % Proximities
  global d_min;
  global d_max;
  global ptol;
  global otol;
  global omega_v;
  global epsilon_omega;

  % Global clock
  global global_clock;

  % The sampling time
  global T;

  % The upper threshold within the time-horizon for which the open loop
  % solution of the counterpart agent is taken into consideration
  global point_in_horizon_ceiling;

  % Lipschitz constants
  global L_g;
  global L_v;

  % The magnitude of the disturbance
  global disturbance;

  % Flags for agents arriving inside their terminal regions
  global agent_1_arrived_at_terminal_region;
  global agent_2_arrived_at_terminal_region;
  global agent_3_arrived_at_terminal_region;
  global deactivate_constraints;


  %% NMPC Parameters

  total_iterations = 1000;
  mpciterations  = 1;
  N              = 6;       % length of Horizon
  T              = 0.1;     % sampling time
  tol_opt        = 1e-8;
  opt_option     = 0;
  iprint         = 5;
  type           = 'differential equation';
  atol_ode_real  = 1e-12;
  rtol_ode_real  = 1e-12;
  atol_ode_sim   = 1e-4;
  rtol_ode_sim   = 1e-4;


  % init Agent 1
  tmeasure_1     = 0.0;
  x_init_1       = [-6, 3.5, 0];
  des_1          = [6, 3.5, 0];
  xmeasure_1     = x_init_1 - des_1;
  u0_1           = 10*ones(num_inputs, N);

  global xX_1;
  tT_1 = [tmeasure_1];
  xX_1 = [xmeasure_1];
  uU_1           = [];


  % init Agent 2
  tmeasure_2     = 0.0;
  x_init_2       = [-6, 2.3, 0];
  des_2          = [6, 2.3, 0];
  xmeasure_2     = x_init_2 - des_2;
  u0_2           = 10*ones(num_inputs, N);

  global xX_2;
  tT_2 = [tmeasure_2];
  xX_2 = [xmeasure_2];
  uU_2           = [];

  % init Agent 3
  tmeasure_3     = 0.0;
  x_init_3       = [-6, 4.7, 0];
  des_3          = [6, 4.7, 0];
  xmeasure_3     = x_init_3 - des_3;
  u0_3           = 10*ones(num_inputs, N);

  global xX_3;
  tT_3 = [tmeasure_3];
  xX_3 = [xmeasure_3];
  uU_3           = [];

  % Penalty matrices
  R              = 0.005 * blkdiag(5,1);

  Q              = 0.7*[0.7349    0.0506    0.0034;
                    0.1260    0.7349    0.0710;
                    0.2113    0.1336    0.7349];

  P              = 0.5*[0.7349    0.0506    0.0034;
                    0.1260    0.7349    0.0710;
                    0.2113    0.1336    0.7349];


  % Input bounds
  u_abs          = 8;
  u_max          = u_abs;
  u_min          = -u_abs;


  % obstacles: x_c, y_c, r
  obs_0          = [0, 2.0, 1;
                    0, 5.5, 1];

  % The obstacles as a series of points ----------------------------------------
  o_t = 0:0.01*pi:2*pi;
  o_x_1 = obs_0(1,1) + obs_0(1,3) * cos(o_t);
  o_y_1 = obs_0(1,2) + obs_0(1,3) * sin(o_t);
  o_x_2 = obs_0(2,1) + obs_0(2,3) * cos(o_t);
  o_y_2 = obs_0(2,2) + obs_0(2,3) * sin(o_t);

  obs_1 = [o_x_1; o_y_1];
  obs_2 = [o_x_2; o_y_2];
  obs = {obs_1, obs_2};

  % The visible points of the objects to agent 1
  global visible_obstacles_1;
  global visible_obstacles_2;
  global visible_obstacles_3;

  % The visible obstacles per iteration
  visible_obstacles_all_1 = cell(1, total_iterations);
  visible_obstacles_all_2 = cell(1, total_iterations);
  visible_obstacles_all_3 = cell(1, total_iterations);

  % The visible obstacle points per iteration
  visible_obstacle_points_all_1 = cell(1, total_iterations);
  visible_obstacle_points_all_2 = cell(1, total_iterations);
  visible_obstacle_points_all_3 = cell(1, total_iterations);

  % The spatial sensing range of the agents
  b              = [4;4;4];


  % radii of agents
  r              = [0.5; 0.5; 0.5];

  % Proximity tolerance between agents
  ptol           = 0.01;

  % Proximity tolerance between an agent and obstacles
  otol           = 0.01;

  % Distance bounds
  d_min          = r(1) + r(2) + ptol;
  d_max          = 2 * (r(1) + r(2)) + ptol;

  % The amplitude of the disturbance
  disturbance    = 0.10

  % Sanity check
%   disturbance = 0;

  % Terminal cost tolerance
  omega_v        = 0.05;

  % Lipschitz constants
  L_g            = u_max * sqrt(sum(sum(Q(1:2,1:2))))
  L_v            = 2 * svds(P,1) * omega_v

  % Terminal set bounds
  epsilon_omega  = omega_v^2 * 3 * svds(P,1)
  epsilon_psi    = (L_v / L_g * exp(L_g * (N * T - T)) * (exp(L_g * T) - 1)) * disturbance + epsilon_omega
  omega_psi      = sqrt(epsilon_psi / (3 * svds(P,1)))

  % Flags for agents arriving inside their terminal regions
  agent_1_arrived_at_terminal_region = false;
  agent_2_arrived_at_terminal_region = false;
  agent_3_arrived_at_terminal_region = false;

  deactivate_constraints = agent_1_arrived_at_terminal_region && ...
    agent_2_arrived_at_terminal_region && ...
    agent_3_arrived_at_terminal_region;

  % Initialize global clock
  global_clock = 0.0;


  point_in_horizon_ceiling = 2;


  for k = 1:total_iterations

    fprintf('iteration %d\n', k);

    deactivate_constraints_before = deactivate_constraints;

    deactivate_constraints = agent_1_arrived_at_terminal_region && ...
      agent_2_arrived_at_terminal_region && ...
      agent_3_arrived_at_terminal_region;

    if ~deactivate_constraints_before && deactivate_constraints
      fprintf('Interconstraints deactivated\n');
    end


    % Increment clock
    global_clock = global_clock + T;

    % Solve for agent 3 --------------------------------------------------------
    % Obstacles are unknown but discoverable once within the spatial sensing
    % range of the agent
    [visible_obstacles_3, visible_obstacle_points_3] = ...
      sense_obstacles(xmeasure_3 + des_3, r(3), b(3), obs, obs_0);

    visible_obstacles_all_3{k} = visible_obstacles_3;
    visible_obstacle_points_all_3{k} = visible_obstacle_points_3;

    nmpc_3(@runningcosts_3, @terminalcosts_3, @constraints_3, ...
      @terminalconstraints_3, @linearconstraints_3, @system_ct_3, ...
      @system_ct_3_real, mpciterations, N, T, tmeasure_3, xmeasure_3, u0_3, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_3, @printClosedloopData_3);

    tmeasure_3      = t_3(end);     % new t_0
    x_init_3        = x_3(end,:);   % new x_0
    xmeasure_3      = x_init_3;
    u0_3            = [u_open_loop_3(:,2:size(u_open_loop_3,2)) u_open_loop_3(:,size(u_open_loop_3,2))];
    tT_3 = [tT_3; tmeasure_3];
    xX_3 = [xX_3; xmeasure_3];

    % Store the applied input
    uU_3 = [uU_3, u_3];

    save('xX_3.mat');
    save('uU_3.mat');
    save('tT_3.mat');

    % Indicate whether agent 2 has arrived within its terminal region Omega
    if x_init_3 * P * x_init_3' <= epsilon_psi
      agent_3_arrived_at_terminal_region = true;
    end

    % Solve for agent 1 --------------------------------------------------------
    % Obstacles are unknown but discoverable once within the spatial sensing
    % range of the agent
    [visible_obstacles_1, visible_obstacle_points_1] = ...
      sense_obstacles(xmeasure_1 + des_1, r(1), b(1), obs, obs_0);

    visible_obstacles_all_1{k} = visible_obstacles_1;
    visible_obstacle_points_all_1{k} = visible_obstacle_points_1;

    nmpc_1(@runningcosts_1, @terminalcosts_1, @constraints_1, ...
      @terminalconstraints_1, @linearconstraints_1, @system_ct_1, ...
      @system_ct_1_real, mpciterations, N, T, tmeasure_1, xmeasure_1, u0_1, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_1, @printClosedloopData_1);

    tmeasure_1      = t_1(end);     % new t_0
    x_init_1        = x_1(end,:);   % new x_0
    xmeasure_1      = x_init_1;
    u0_1            = [u_open_loop_1(:,2:size(u_open_loop_1,2)) u_open_loop_1(:,size(u_open_loop_1,2))];
    tT_1 = [tT_1; tmeasure_1];
    xX_1 = [xX_1; xmeasure_1];

    % Store the applied input
    uU_1 = [uU_1, u_1];

    save('xX_1.mat');
    save('uU_1.mat');
    save('tT_1.mat');

    % Indicate whether agent 1 has arrived within its terminal region Omega
    if x_init_1 * P * x_init_1' <= epsilon_psi
      agent_1_arrived_at_terminal_region = true;
    end


    % Solve for agent 2 --------------------------------------------------------
    % Obstacles are unknown but discoverable once within the spatial sensing
    % range of the agent
    [visible_obstacles_2, visible_obstacle_points_2] = ...
      sense_obstacles(xmeasure_2 + des_2, r(2), b(2), obs, obs_0);

    visible_obstacles_all_2{k} = visible_obstacles_2;
    visible_obstacle_points_all_2{k} = visible_obstacle_points_2;

    nmpc_2(@runningcosts_2, @terminalcosts_2, @constraints_2, ...
      @terminalconstraints_2, @linearconstraints_2, @system_ct_2, ...
      @system_ct_2_real, mpciterations, N, T, tmeasure_2, xmeasure_2, u0_2, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_2, @printClosedloopData_2);

    tmeasure_2      = t_2(end);     % new t_0
    x_init_2        = x_2(end,:);   % new x_0
    xmeasure_2      = x_init_2;
    u0_2            = [u_open_loop_2(:,2:size(u_open_loop_2,2)) u_open_loop_2(:,size(u_open_loop_2,2))];
    tT_2 = [tT_2; tmeasure_2];
    xX_2 = [xX_2; xmeasure_2];

    % Store the applied input
    uU_2 = [uU_2, u_2];

    save('xX_2.mat');
    save('uU_2.mat');
    save('tT_2.mat');

    % Indicate whether agent 2 has arrived within its terminal region Omega
    if x_init_2 * P * x_init_2' <= epsilon_psi
      agent_2_arrived_at_terminal_region = true;
    end


    save('variables.mat');

  end

  toc;
end



%% Running Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = runningcosts_1(t_1, e_1, u_1)
  global Q;
  global R;

  e_1=e_1';

  Q_1 = Q;
  R_1 = R;

  cost_1 = e_1'*Q_1*e_1 + u_1'*R_1*u_1;
end

function cost_2 = runningcosts_2(t_2, e_2, u_2)
  global Q;
  global R;

  e_2=e_2';

  Q_2 = Q;
  R_2 = R;

  cost_2 = e_2'*Q_2*e_2 + u_2'*R_2*u_2;
end

function cost_3 = runningcosts_3(t_3, e_3, u_3)
  global Q;
  global R;

  e_3=e_3';

  Q_3 = Q;
  R_3 = R;

  cost_3 = e_3'*Q_3*e_3 + u_3'*R_3*u_3;
end


%% Terminal Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = terminalcosts_1(t_1, e_1)
  global P;

  e_1 = e_1';

  P_1 = P;

  cost_1 = e_1'*P_1*e_1;
end

function cost_2 = terminalcosts_2(t_2, e_2)
  global P;

  e_2 = e_2';

  P_2 = P;

  cost_2 = e_2'*P_2*e_2;
end

function cost_3 = terminalcosts_3(t_3, e_3)
  global P;

  e_3 = e_3';

  P_3 = P;

  cost_3 = e_3'*P_3*e_3;
end

%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t_1, e_1, u_1)

  global des_1;
  global des_2;
  global des_3;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_2;
  global x_open_loop_3;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global deactivate_constraints;
  global visible_obstacles_1;
  global xX_2;
  global xX_3;

  %disp('in constraints_1')
  n_2 = size(x_open_loop_2, 1);
  n_3 = size(x_open_loop_3, 1);

  c = [];
  ceq = [];

  ball_t_1 = disturbance / L_g * (exp(L_g * (t_1 - global_clock)) - 1);

  th = ball_t_1 / sqrt(3 * svds(Q,1));

  x = e_1(1) + th;
  y = e_1(2) + th;
  z = e_1(3) + th;

  % Avoid collision with obstacles -- max
  for i = 1:size(visible_obstacles_1,1)
    c = [c, (visible_obstacles_1(i,3) + r(1) + otol)^2 - ...
      ((x+des_1(1) - visible_obstacles_1(i,1))^2 + (y+des_1(2) - visible_obstacles_1(i,2))^2)];
  end

  c = [c, z + des_1(3) - pi];
  c = [c, -z - des_1(3) - pi];

  x = e_1(1) - th;
  y = e_1(2) - th;
  z = e_1(3) - th;

  % Avoid collision with obstacles -- min
  for i = 1:size(visible_obstacles_1,1)
    c = [c, (visible_obstacles_1(i,3) + r(1) + otol)^2 - ...
      ((x+des_1(1) - visible_obstacles_1(i,1))^2 + (y+des_1(2) - visible_obstacles_1(i,2))^2)];
  end

  c = [c, z + des_1(3) - pi];
  c = [c, -z - des_1(3) - pi];

  % The index pointing to the current configuration of agent 2 from the
  % perspective of agent 1.
  % index 2 is the current configuration of agents 2,3
  point_in_horizon = 1 + int8( (t_1 - global_clock) / T );

   if deactivate_constraints == false
    if n_2 > 0

      x_open_loop_2_cut = xX_2(end-1:end,:);
      x_open_loop_2_cut = [x_open_loop_2_cut; x_open_loop_2(3:end,:)];

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance between the two agents -- max
        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) + 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c = [c, d_min - dist];

        % Maintain connectivity with agent 2 along the entire horizon
        c = [c, dist - d_max];

        % The distance between the two agents -- min
        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) - 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c = [c, d_min - dist];

        % Maintain connectivity with agent 2 along the entire horizon
        c = [c, dist - d_max];

      end
    end


    if n_3 > 0

      x_open_loop_3_cut = xX_3(end-1:end,:);
      x_open_loop_3_cut = [x_open_loop_3_cut; x_open_loop_3(3:end,:)];

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance between the two agents -- max
        dist_x = e_1(1) + des_1(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) + 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c = [c, d_min - dist];

        % Maintain connectivity with agent 3 along the entire horizon
        c = [c, dist - d_max];

        % The distance between the two agents -- min
        dist_x = e_1(1) + des_1(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) - 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c = [c, d_min - dist];

        % Maintain connectivity with agent 3 along the entire horizon
        c = [c, dist - d_max];

      end
    end
  end
end




function [c,ceq] = constraints_2(t_2, e_2, u_2)

  global des_1;
  global des_2;
  global des_3;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global x_open_loop_3;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global deactivate_constraints;
  global visible_obstacles_2;
  global xX_1;
  global xX_3


  %disp('in constraints_2')
  n_1 = size(x_open_loop_1, 1);
  n_3 = size(x_open_loop_3, 1);

  c = [];
  ceq = [];

  ball_t_2 = disturbance / L_g * (exp(L_g * (t_2 - global_clock)) - 1);

  th = ball_t_2 / sqrt(3 * svds(Q,1));

  x = e_2(1) + th;
  y = e_2(2) + th;
  z = e_2(3) + th;

  % Avoid collision with obstacles -- max
  for i = 1:size(visible_obstacles_2,1)
    c = [c, (visible_obstacles_2(i,3) + r(2) + otol) - ...
      sqrt((x+des_2(1) - visible_obstacles_2(i,1))^2 + (y+des_2(2) - visible_obstacles_2(i,2))^2)];
  end

  c = [c, z + des_2(3) - pi];
  c = [c, -z - des_2(3) - pi];


  x = e_2(1) - th;
  y = e_2(2) - th;
  z = e_2(3) - th;

  % Avoid collision with obstacles -- min
  for i = 1:size(visible_obstacles_2,1)
    c = [c, (visible_obstacles_2(i,3) + r(2) + otol) - ...
      sqrt((x+des_2(1) - visible_obstacles_2(i,1))^2 + (y+des_2(2) - visible_obstacles_2(i,2))^2)];
  end

  c = [c, z + des_2(3) - pi];
  c = [c, -z - des_2(3) - pi];

  % The index pointing to the current configuration of agent 1 from the
  % perspective of agent 2.
  % index 2 is the current configuration of agents 1,3
  point_in_horizon = 1 + int8( (t_2 - global_clock) / T );

  if deactivate_constraints == false

    if n_1 > 0

      x_open_loop_1_cut = xX_1(end-1:end,:);
      x_open_loop_1_cut = [x_open_loop_1_cut; x_open_loop_1(3:end,:)];

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance between the two agents -- max
        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) + 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c = [c, d_min - dist];

        % Maintain connectivity with agent 1 along the entire horizon
        c = [c, dist - d_max];

        % The distance between the two agents -- min
        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) - 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c = [c, d_min - dist];

        % Maintain connectivity with agent 1 along the entire horizon
        c = [c, dist - d_max];

      end
    end

    if n_3 > 0

      x_open_loop_3_cut = xX_3(end-1:end,:);
      x_open_loop_3_cut = [x_open_loop_3_cut; x_open_loop_3(3:end,:)];

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance between the two agents
        dist_x = e_2(1) + des_2(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) + 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c = [c, d_min - dist];

        % The distance between the two agents
        dist_x = e_2(1) + des_2(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) - 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c = [c, d_min - dist];

      end
    end
  end
end



function [c,ceq] = constraints_3(t_3, e_3, u_3)

  global des_1;
  global des_2;
  global des_3;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global x_open_loop_2;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global deactivate_constraints;
  global visible_obstacles_3;
  global xX_1;
  global xX_2;


  %disp('in constraints_3')
  n_1 = size(x_open_loop_1, 1);
  n_2 = size(x_open_loop_2, 1);

  c = [];
  ceq = [];


  ball_t_3 = disturbance / L_g * (exp(L_g * (t_3 - global_clock)) - 1);

  th = ball_t_3 / sqrt(3 * svds(Q,1));

  x = e_3(1) + th;
  y = e_3(2) + th;
  z = e_3(3) + th;


  % Avoid collision with obstacles -- max
  for i = 1:size(visible_obstacles_3,1)
    c = [c, (visible_obstacles_3(i,3) + r(2) + otol) - ...
      sqrt((x+des_3(1) - visible_obstacles_3(i,1))^2 + (y+des_3(2) - visible_obstacles_3(i,2))^2)];
  end

  c = [c, z + des_3(3) - pi];
  c = [c, -z - des_3(3) - pi];

  x = e_3(1) - th;
  y = e_3(2) - th;
  z = e_3(3) - th;


  % Avoid collision with obstacles -- max
  for i = 1:size(visible_obstacles_3,1)
    c = [c, (visible_obstacles_3(i,3) + r(2) + otol) - ...
      sqrt((x+des_3(1) - visible_obstacles_3(i,1))^2 + (y+des_3(2) - visible_obstacles_3(i,2))^2)];
  end

  c = [c, z + des_3(3) - pi];
  c = [c, -z - des_3(3) - pi];


  % The index pointing to the current configuration of agent 3 from the
  % perspective of agents 2,3.
  % index 2 is the current configuration of agent 1
  point_in_horizon = 1 + int8( (t_3 - global_clock) / T );

  if deactivate_constraints == false

    if n_1 > 0

      x_open_loop_1_cut = xX_1(end-1:end,:);
      x_open_loop_1_cut = [x_open_loop_1_cut; x_open_loop_1(3:end,:)];

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance between the two agents -- max
        dist_x = e_3(1) + des_3(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) + 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c = [c, d_min - dist];
  %
        % Maintain connectivity with agent 1 along the entire horizon
        c = [c, dist - d_max];

        % The distance between the two agents -- min
        dist_x = e_3(1) + des_3(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) - 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c = [c, d_min - dist];
  %
        % Maintain connectivity with agent 1 along the entire horizon
        c = [c, dist - d_max];

      end
    end

    if n_2 > 0

      x_open_loop_2_cut = xX_2(end-1:end,:);
      x_open_loop_2_cut = [x_open_loop_2_cut; x_open_loop_2(3:end,:)];

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance between the two agents -- max
        dist_x = e_3(1) + des_3(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) + 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c = [c, d_min - dist];

        % The distance between the two agents -- min
        dist_x = e_3(1) + des_3(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) - 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c = [c, d_min - dist];

      end
    end
  end
end


%% Terminal Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = terminalconstraints_1(t_1, e_1)

  global omega_v;

  c = [];
  ceq = [];


  c = [c, e_1(1) - omega_v];
  c = [c, -e_1(1) - omega_v];
  c = [c, e_1(2) - omega_v];
  c = [c, -e_1(2) - omega_v];
  c = [c, e_1(3) - omega_v];
  c = [c, -e_1(3) - omega_v];
end

function [c,ceq] = terminalconstraints_2(t_2, e_2)

  global omega_v;

  c = [];
  ceq = [];


  c = [c, e_2(1) - omega_v];
  c = [c, -e_2(1) - omega_v];
  c = [c, e_2(2) - omega_v];
  c = [c, -e_2(2) - omega_v];
  c = [c, e_2(3) - omega_v];
  c = [c, -e_2(3) - omega_v];

end

function [c,ceq] = terminalconstraints_3(t_3, e_3)

  global omega_v;

  c = [];
  ceq = [];


  c = [c, e_3(1) - omega_v];
  c = [c, -e_3(1) - omega_v];
  c = [c, e_3(2) - omega_v];
  c = [c, -e_3(2) - omega_v];
  c = [c, e_3(3) - omega_v];
  c = [c, -e_3(3) - omega_v];

end

%% Control Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = linearconstraints_1(t_1, x_1, u_1)
  global u_max;
  global u_min;

  A   = [];
  b   = [];
  Aeq = [];
  beq = [];

  % u constraints
  lb  = u_min;
  ub  = u_max;
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints_2(t_2, x_2, u_2)
  global u_max;
  global u_min;

  A   = [];
  b   = [];
  Aeq = [];
  beq = [];

  % u constraints
  lb  = u_min;
  ub  = u_max;
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints_3(t_3, x_3, u_3)
  global u_max;
  global u_min;

  A   = [];
  b   = [];
  Aeq = [];
  beq = [];

  % u constraints
  lb  = u_min;
  ub  = u_max;
end


%% Output Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function printHeader_1()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_1(t)      u2_1(t)      e1_1(t)     e1_2(t)     e1_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end

function printHeader_2()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_2(t)      u2_2(t)      e2_1(t)     e2_2(t)     e2_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end

function printHeader_3()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_3(t)      u2_3(t)      e3_1(t)     e3_2(t)     e3_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end

function printClosedloopData_1(mpciter, u_1, e_1, t_Elapsed_1)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f  %+11.6f', ...
    mpciter, u_1(1), u_1(2), e_1(1), e_1(2), e_1(3), t_Elapsed_1);
end

function printClosedloopData_2(mpciter, u_2, e_2, t_Elapsed_2)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_2(1), u_2(2), e_2(1), e_2(2), e_2(3), t_Elapsed_2);
end

function printClosedloopData_3(mpciter, u_3, e_3, t_Elapsed_3)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_3(1), u_3(2), e_3(1), e_3(2), e_3(3), t_Elapsed_3);
end
