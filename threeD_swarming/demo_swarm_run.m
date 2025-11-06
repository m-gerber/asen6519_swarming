function demo_swarm_run(n, Ls, goal, T, dt, P0, V0, obstacles, obs_params)

    % DEMO_SWARM_RUN  Minimal 3D swarm simulator with spacing,
    % heading, altitude, goal tracking, and obstacle avoidance via
    % short-range repulsive fields (e.g., finite cylinders).

    %
    % DEMO_SWARM_RUN(N, Ls, goal, T, dt, P0, V0) runs the simulation with
    % the provided parameters. Call with fewer inputs (or []) to use the
    % defaults shown below. Exports CSVs in ./swarm_output/.

    %==============================================================|
    % Default parameters
    if nargin < 1 || isempty(n),   n   = 5;               end
    if nargin < 2 || isempty(Ls),  Ls  = 15.0;            end % desired mean separation [m]
    if nargin < 3 || isempty(goal),goal= [200; 50; 40];   end % final goal position [m]
    if nargin < 4 || isempty(T),   T   = 60;              end % total sim time [s]
    if nargin < 5 || isempty(dt),  dt  = 0.05;            end % time step [s]

    % Initial states (n x 3 each)
    if nargin < 6 || isempty(P0)
        side = ceil(sqrt(n));
        spacing = 10;
        [xx, yy] = meshgrid(0:spacing:(side-1)*spacing);
        xy = [xx(:), yy(:)];
        P0 = [xy(1:n,:), 20*ones(n,1)];
    end
    if nargin < 7 || isempty(V0)
        V0 = repmat([4 0 0], n, 1);
    end

    % Obstacles and obstacle parameters (optional)
    if nargin < 8 || isempty(obstacles)
        obstacles = []; % no obstacles by default
    end
    if nargin < 9 || isempty(obs_params)
        obs_params.k_o    = 40.0;  % repulsion gain [m^2/s^2]
        obs_params.d_safe = 12.0;  % influence radius [m]
        obs_params.fmax   = 10.0;  % accel cap [m/s^2]
    end

    % Normalize obstacle structs
    for kk = 1:numel(obstacles)
        if ~isfield(obstacles(kk),'type'),   obstacles(kk).type = 'cylinder'; end
        if strcmpi(obstacles(kk).type,'cylinder')
            need = {'xy','radius','zmin','zmax'};
            for f = need
                if ~isfield(obstacles(kk), f{1})
                    error('Cylinder obstacle %d missing field "%s".', kk, f{1});
                end
            end
        end
    end
    % Normalize obs_params fields
    def = struct('k_o',40.0,'d_safe',12.0,'fmax',10.0);
    fn = fieldnames(def);
    for ii = 1:numel(fn)
        f = fn{ii};
        if ~isfield(obs_params,f) || isempty(obs_params.(f))
            obs_params.(f) = def.(f);
        end
    end


    
    % Safety checks
    assert(size(P0,1)==n && size(V0,1)==n && size(P0,2)==3 && size(V0,2)==3, ...
        'P0 and V0 must be n-by-3');

    % Controller parameters
    k_g = 0.02;       % goal attraction
    k_s = 0.8;        % separation-length regulation
    k_a = 0.6;        % heading alignment
    k_h = 0.5;        % altitude tracking
    c_d = 0.25;       % linear damping on velocity
    goal_tol = 5.0;   % considered "at goal" when within this radius [m]
    
    amax = 6.0;       % accel limit [m/s^2]
    vmax = 15.0;      % speed limit [m/s]
    eps_d = 1e-3;     % small distance epsilon
    
    % Altitude target: fixed or initial mean
    h_star = goal(3);                 % or set to a fixed number

    % Preallocate logs
    steps = floor(T/dt)+1;
    P     = zeros(n,3,steps); V = zeros(n,3,steps);

    P(:,:,1) = P0; V(:,:,1) = V0;

    % Main loop (RK4 on double-integrator)
    g = goal(:);
    settle_steps_required = 20; % number of consecutive in-tolerance steps to declare done
    settle_counter = 0;
    for k = 1:steps-1
        p = P(:,:,k); v = V(:,:,k);
    
        % RK4 stages on [p; v]
        [dp1, dv1] = swarm_derivs(p, v);
        [dp2, dv2] = swarm_derivs(p + 0.5*dt*dp1, v + 0.5*dt*dv1);
        [dp3, dv3] = swarm_derivs(p + 0.5*dt*dp2, v + 0.5*dt*dv2);
        [dp4, dv4] = swarm_derivs(p + dt*dp3,    v + dt*dv3);
    
        p_next = p + (dt/6)*(dp1 + 2*dp2 + 2*dp3 + dp4);
        v_next = v + (dt/6)*(dv1 + 2*dv2 + 2*dv3 + dv4);
    
        % Speed limit
        spd = vecnorm(v_next,2,2);
        scale = min(1, vmax./max(spd,1e-9));
        v_next = v_next .* scale;
    
        P(:,:,k+1) = p_next;
        V(:,:,k+1) = v_next;

        dist_next = vecnorm(p_next - g', 2, 2);
        if all(dist_next <= goal_tol)
            settle_counter = settle_counter + 1;
        else
            settle_counter = 0;
        end

        if settle_counter >= settle_steps_required
            P = P(:,:,1:k+1);
            V = V(:,:,1:k+1);
            steps = k + 1;
            break;
        end
    end

    % Goal diagnostics
    final_pos = P(:,:,end);
    dist_to_goal = vecnorm(final_pos - g', 2, 2);
    max_dist = max(dist_to_goal);
    min_dist = min(dist_to_goal);
    mean_dist = mean(dist_to_goal);
    reached_goal = max_dist <= goal_tol;
    if reached_goal
        fprintf('All drones reached the goal within %.2f m. Min/Mean/Max distances: %.2f / %.2f / %.2f m\n', ...
            goal_tol, min_dist, mean_dist, max_dist);
    else
        fprintf('Goal not reached: Min/Mean/Max distances to goal = %.2f / %.2f / %.2f m (threshold %.2f m)\n', ...
            min_dist, mean_dist, max_dist, goal_tol);
    end

    % Export logs
    outdir = fullfile(pwd,'swarm_output');
    if ~exist(outdir,'dir'), mkdir(outdir); end
    
    t = (0:steps-1)'*dt;
    for i = 1:n
        M = [t, squeeze(P(i,1,:)), squeeze(P(i,2,:)), squeeze(P(i,3,:)), ...
                squeeze(V(i,1,:)), squeeze(V(i,2,:)), squeeze(V(i,3,:))];
        tbl = array2table(M, 'VariableNames', ...
            {'t_s','x_m','y_m','z_m','vx_ms','vy_ms','vz_ms'});
        writetable(tbl, fullfile(outdir, sprintf('drone_%02d.csv',i)));
    end
    
    % Combined file
    Pflat = reshape(P, [n*steps, 3]);
    Vflat = reshape(V, [n*steps, 3]);
    idcol = repelem((1:n)', steps, 1);
    tcol  = repmat(t, n, 1);
    All = [idcol, tcol, Pflat, Vflat];
    tblAll = array2table(All, 'VariableNames', ...
        {'id','t_s','x_m','y_m','z_m','vx_ms','vy_ms','vz_ms'});
    writetable(tblAll, fullfile(outdir, 'swarm_all.csv'));
    
    fprintf('Exported %d per-drone files and combined file to %s\n', n, outdir);

    % Optional quick plot
    figure; hold on; grid on; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    for i = 1:n
        plot3(squeeze(P(i,1,:)), squeeze(P(i,2,:)), squeeze(P(i,3,:)));
    end
    plot3(g(1), g(2), g(3), 'ko', 'MarkerFaceColor','y'); % goal

    % Draw obstacles (solid blue cylinders)
    if ~isempty(obstacles)
        for kk = 1:numel(obstacles)
            obs = obstacles(kk);
            switch lower(obs.type)
                case 'cylinder'
                    cx = obs.xy(1); cy = obs.xy(2);
                    R  = obs.radius; z0 = obs.zmin; z1 = obs.zmax;

                    th = linspace(0, 2*pi, 60);
                    z  = [z0 z1];
                    [TH, ZZ] = meshgrid(th, z);
                    XX = cx + R*cos(TH);
                    YY = cy + R*sin(TH);

                    % Lateral surface
                    s = surf(XX, YY, ZZ);
                    set(s, 'FaceColor','b', 'EdgeColor','none', 'FaceAlpha',0.35);

                    % Caps
                    xc = cx + R*cos(th);  yc = cy + R*sin(th);
                    fill3(xc, yc, z0*ones(size(th)), 'b', 'EdgeColor','none', 'FaceAlpha',0.35);
                    fill3(xc, yc, z1*ones(size(th)), 'b', 'EdgeColor','none', 'FaceAlpha',0.35);
            end
        end
        % Optional cosmetics
        camlight headlight; material dull;
    end

    title('Swarm trajectories'); view(35,25);

    %Nested functions
    function [dp, dv] = swarm_derivs(p, v)
        % p, v are n-by-3
        % Goal term
        a_goal = k_g * (g' - p);

        % Alignment term
        vbar = mean(v,1);
        a_align = k_a * (vbar - v);

        % Altitude term
        a_alt = [zeros(n,2), k_h*(h_star - p(:,3))];


        % Obstacle repulsion term
        if isempty(obstacles) || obs_params.d_safe <= 0
            a_obs = zeros(n,3);
        else
            a_obs = obstacle_repulsion(p, obstacles, obs_params);
        end

        % Separation-length regulation
        a_sep = zeros(n,3);
        for ii = 1:n
            acc = [0 0 0];
            for jj = 1:n
                if jj==ii, continue; end
                rij = p(jj,:) - p(ii,:);
                dij = norm(rij);
                if dij < eps_d, continue; end
                rhat = rij / dij;
                acc = acc + (dij - Ls) * rhat;
            end
            a_sep(ii,:) = (k_s/(n-1)) * acc;
        end

        a = a_goal + a_align + a_sep + a_alt + a_obs - c_d * v;

        % Acceleration saturation
        anorm = vecnorm(a,2,2);
        scl = min(1, amax ./ max(anorm,1e-9));
        a = a .* scl;

        dp = v;
        dv = a;
    end
end
