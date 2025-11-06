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


obs_params.k_o    = 40.0;   % repulsion gain, larger = stronger repulsion
                            % units = m/s^2
obs_params.d_safe = 18.0;   %Radius of influence - range at which obstacle avoidance begins [m]
obs_params.fmax   = 10.0;   %Limit on maximum obstacle avoidance acceleration [m/s^2]

if run_w_obstacles
    obstacles(1).type   = 'cylinder';
    obstacles(1).xy     = [100, 100];
    obstacles(1).radius = 6.0;
    obstacles(1).zmin   = 0.0;
    obstacles(1).zmax   = 120.0;   % higher than goal altitude

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














