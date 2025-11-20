clear; close all;clc
n  = 5;                   % number of drones (>=3)
Ls = 3.0;                 % desired mean separation [m]
goal = [200; 200; 50];    % final goal position [m]
T   = 45.0;               % total sim time [s]
dt  = 0.05;               % time step [s]

run_w_obstacles = true;
%==============================================================|

% Initial states (n x 3 each). Provide your own values here:
P0 = [  4.0   0.0   20.0;
        -4.0  0.0   20.0;
        8.0   0.0   20.0;
        -8.0  0.0   20.0;
        12.0   0.0   20.0];     % positions [m]
V0 = repmat([4 0 0], n, 1);  % velocities [m/s]

% Cylinder centered on the straight path (45° line), tall enough to block overflight
obstacles(1).type   = 'cylinder';
obstacles(1).xy     = [100, 100];
obstacles(1).radius = 6.0;     
obstacles(1).zmin   = 0.0;
obstacles(1).zmax   = 120.0;

% Planar wall (rectangular panel) to stress-test avoidance logic
obstacles(2).type    = 'wall';
obstacles(2).center  = [140, 120, 40];   % center of the panel [m]
obstacles(2).normal  = [-1, 0, 0];       % outward normal direction
obstacles(2).width   = 45.0;             % span along local u-axis [m]
obstacles(2).height  = 60.0;             % span along local v-axis [m]
obstacles(2).up      = [0, 0, 1];        % optional reference to keep panel upright
obstacles(2).thickness = 8.0;            % finite thickness so drones must route around
obstacles(2).d_safe    = 25.0;           % extend avoidance bubble for the wall
obstacles(2).fmax      = 15.0;           % allow stronger braking near the wall


obs_params.k_o    = 40.0;   % repulsion gain, larger = stronger repulsion
                            % units = m/s^2
obs_params.d_safe = 18.0;   %Radius of influence - range at which obstacle avoidance begins [m]
obs_params.fmax   = 10.0;   %Limit on maximum obstacle avoidance acceleration [m/s^2]

if run_w_obstacles
    % obstacles(1).type   = 'cylinder';
    % obstacles(1).xy     = [100, 100];
    % obstacles(1).radius = 6.0;
    % obstacles(1).zmin   = 0.0;
    % obstacles(1).zmax   = 120.0;   % higher than goal altitude

    obstacles(2).type    = 'wall';
    obstacles(2).center  = [140, 120, 40];
    obstacles(2).normal  = [-1, 0, 0];
    obstacles(2).width   = 45.0;
    obstacles(2).height  = 60.0;
    obstacles(2).up      = [0, 0, 1];
    obstacles(2).thickness = 8.0;
    obstacles(2).d_safe    = 25.0;
    obstacles(2).fmax      = 15.0;

    obs_params.k_o    = 40.0;      % repulsion gain
    obs_params.d_safe = 18.0;      % influence radius
    obs_params.fmax   = 10.0;      % accel cap

    demo_swarm_run(n, Ls, goal, T, dt, P0, V0, obstacles, obs_params);

    data.csv_dir   = fullfile(pwd,'swarm_output');
    data.goal      = goal;  % reuse the variable
    data.obstacles = obstacles;
    data.obs_style = struct('color','b','alpha',0.35,'edgecolor','none');
    animate_flight_paths(2, data);
else
    demo_swarm_run(n, Ls, goal, T, dt, P0, V0, [], []);
    animate_flight_paths(2, struct('csv_dir', fullfile(pwd,'swarm_output'), 'goal', goal));
end














