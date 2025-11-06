function animate_flight_paths(speed_up, data)
    % ANIMATE_FLIGHT_PATHS  Render and export a swarm flight video.
    % speed_up: playback speed factor (2 => twice as fast as real time)
    % data: struct with either preloaded arrays or CSV directory info. Fields:
    %   positions (n x 3 x steps) and time (steps x 1) OR
    %   csv_dir (directory containing drone_*.csv files)
    %   goal (optional 3x1 goal vector)
    %   obstacles (optional struct array: e.g., cylinders with fields
    %     type='cylinder', xy=[cx,cy], radius, zmin, zmax)
    %   obs_style (optional struct: color, alpha, edgecolor)
    %   output_file (optional video path, default swarm_animation.mp4)

    if nargin < 1 || isempty(speed_up), speed_up = 1; end
    if nargin < 2, data = struct(); end
    validateattributes(speed_up, {'numeric'}, {'scalar','real','positive'});

    % Load positions/time
    if isfield(data,'positions') && isfield(data,'time')
        positions = data.positions;
        time_vec = data.time(:);
        validateattributes(positions, {'numeric'}, {'3d','nonnan','finite'});
        validateattributes(time_vec, {'numeric'}, {'vector','increasing'});
        n = size(positions,1);
        steps = numel(time_vec);
        assert(size(positions,2)==3 && size(positions,3)==steps, ...
            'positions must be n x 3 x steps with steps matching numel(time).');
    else
        if ~isfield(data,'csv_dir') || isempty(data.csv_dir)
            data.csv_dir = fullfile(pwd,'swarm_output');
        end
        [positions, time_vec] = load_swarm_csv(data.csv_dir);
        n = size(positions,1);
        steps = numel(time_vec);
    end

    % Optional goal
    if isfield(data,'goal') && ~isempty(data.goal)
        goal = data.goal(:)';  if numel(goal) ~= 3, error('Goal must be 3 elements.'); end
    else
        goal = [];
    end

    % Optional obstacles + style
    if isfield(data,'obstacles') && ~isempty(data.obstacles)
        obstacles = data.obstacles;
    else
        obstacles = [];
    end
    style = struct('color','b','alpha',0.35,'edgecolor','none');
    if isfield(data,'obs_style') && ~isempty(data.obs_style)
        fns = fieldnames(data.obs_style);
        for k=1:numel(fns), style.(fns{k}) = data.obs_style.(fns{k}); end
    end

    % Output path
    if isfield(data,'output_file') && ~isempty(data.output_file)
        output_path = data.output_file;
    else
        has_obs = isfield(data,'obstacles') && ~isempty(data.obstacles);
        tag = 'no_obs';
        if has_obs, tag = 'obs'; end
        % Timestamp to avoid overwriting previous runs
        ts = datestr(now,'yyyymmdd_HHMMSS');  % e.g., 20250314_093012
        output_path = fullfile(pwd, sprintf('swarm_animation_%s_%s.mp4', tag, ts));
    end


    % Frame pacing
    dt = median(diff(time_vec));
    frame_rate_desired = speed_up / dt;
    max_fps = 60;
    if frame_rate_desired <= max_fps
        frame_stride = 1; frame_rate = frame_rate_desired;
    else
        frame_stride = ceil(frame_rate_desired / max_fps);
        frame_rate = frame_rate_desired / frame_stride;
    end
    frame_indices = 1:frame_stride:steps;

    % Data ranges
    xdata = squeeze(positions(:,1,:));
    ydata = squeeze(positions(:,2,:));
    zdata = squeeze(positions(:,3,:));
    xrange = [min(xdata,[],'all'), max(xdata,[],'all')];
    yrange = [min(ydata,[],'all'), max(ydata,[],'all')];
    zrange = [min(zdata,[],'all'), max(zdata,[],'all')];

    % Expand ranges to include obstacles
    if ~isempty(obstacles)
        [ox, oy, oz] = obstacle_bounds(obstacles);
        xrange = [min([xrange(1), ox(1)]), max([xrange(2), ox(2)])];
        yrange = [min([yrange(1), oy(1)]), max([yrange(2), oy(2)])];
        zrange = [min([zrange(1), oz(1)]), max([zrange(2), oz(2)])];
    end

    margin = 0.05;
    xrange = expand_range(xrange, margin);
    yrange = expand_range(yrange, margin);
    zrange = expand_range(zrange, margin);

    % Figure setup
    colors = lines(n);
    fig = figure('Name','Swarm Animation','Color','w');
    ax = axes('Parent',fig);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    view(ax,35,25);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
    xlim(ax,xrange); ylim(ax,yrange); zlim(ax,zrange);
    title(ax,'Swarm trajectories');

    % Floor
    [Xplane, Yplane] = meshgrid([xrange(1), xrange(2)], [yrange(1), yrange(2)]);
    Zplane = zrange(1) * ones(size(Xplane));
    surf(ax, Xplane, Yplane, Zplane, 'FaceColor',[0.85 0.85 0.85], ...
        'EdgeColor','none', 'FaceAlpha',0.6);

    % Obstacles (draw once; static)
    if ~isempty(obstacles)
        draw_obstacles(ax, obstacles, style);
        camlight(ax,'headlight'); material(ax,'dull');
    end

    % Paths and markers
    traj = gobjects(n,1);
    markers = gobjects(n,1);
    for ii = 1:n
        traj(ii) = plot3(ax, xdata(ii,1), ydata(ii,1), zdata(ii,1), 'Color', colors(ii,:));
        markers(ii) = plot3(ax, xdata(ii,1), ydata(ii,1), zdata(ii,1), 'o', ...
            'MarkerFaceColor', colors(ii,:), 'MarkerEdgeColor', colors(ii,:));
    end
    if ~isempty(goal)
        plot3(ax, goal(1), goal(2), goal(3), 'ks', 'MarkerFaceColor','y', 'MarkerSize',8);
    end
    time_text = text(ax, 0.02, 0.95, '', 'Units','normalized', 'FontSize',12);

    % Video
    writer = VideoWriter(output_path, 'MPEG-4');
    writer.FrameRate = frame_rate;
    open(writer);

    for idx = 1:numel(frame_indices)
        k = frame_indices(idx);
        for ii = 1:n
            set(traj(ii), 'XData', xdata(ii,1:k), 'YData', ydata(ii,1:k), 'ZData', zdata(ii,1:k));
            set(markers(ii), 'XData', xdata(ii,k), 'YData', ydata(ii,k), 'ZData', zdata(ii,k));
        end
        set(time_text, 'String', sprintf('t = %.2f s', time_vec(k)));
        drawnow limitrate
        frame = getframe(fig);
        writeVideo(writer, frame);
    end

    close(writer);
    fprintf('Saved animation to %s (frames: %d, fps: %.2f)\n', output_path, numel(frame_indices), frame_rate);
end

function draw_obstacles(ax, obstacles, style)
    col = style.color; alp = style.alpha; edg = style.edgecolor;
    for kk = 1:numel(obstacles)
        obs = obstacles(kk);
        switch lower(obs.type)
            case 'cylinder'
                cx = obs.xy(1); cy = obs.xy(2);
                R  = obs.radius; z0 = obs.zmin; z1 = obs.zmax;

                th = linspace(0, 2*pi, 72);
                z  = [z0 z1];
                [TH, ZZ] = meshgrid(th, z);
                XX = cx + R*cos(TH);
                YY = cy + R*sin(TH);

                % Lateral surface
                s = surf(ax, XX, YY, ZZ);
                set(s, 'FaceColor', col, 'EdgeColor', edg, 'FaceAlpha', alp);

                % Caps
                xc = cx + R*cos(th);  yc = cy + R*sin(th);
                fill3(ax, xc, yc, z0*ones(size(th)), col, 'EdgeColor', edg, 'FaceAlpha', alp);
                fill3(ax, xc, yc, z1*ones(size(th)), col, 'EdgeColor', edg, 'FaceAlpha', alp);

            % Extend with spheres/boxes if needed
        end
    end
end

function [xr, yr, zr] = obstacle_bounds(obstacles)
    % Compute bounding box [min max] for obstacles
    xmin = inf; xmax = -inf; ymin = inf; ymax = -inf; zmin = inf; zmax = -inf;
    for kk = 1:numel(obstacles)
        obs = obstacles(kk);
        switch lower(obs.type)
            case 'cylinder'
                cx = obs.xy(1); cy = obs.xy(2); R = obs.radius;
                xmin = min(xmin, cx - R); xmax = max(xmax, cx + R);
                ymin = min(ymin, cy - R); ymax = max(ymax, cy + R);
                zmin = min(zmin, obs.zmin);  zmax = max(zmax, obs.zmax);
        end
    end
    if isinf(xmin) % no obstacles
        xr = [0 0]; yr = [0 0]; zr = [0 0];
    else
        xr = [xmin xmax]; yr = [ymin ymax]; zr = [zmin zmax];
    end
end

function [positions, time_vec] = load_swarm_csv(csv_dir)
    files = dir(fullfile(csv_dir, 'drone_*.csv'));
    if isempty(files)
        error('No drone_*.csv files found in %s', csv_dir);
    end
    [~, order] = sort({files.name});
    files = files(order);
    n = numel(files);
    for ii = 1:n
        tbl = readtable(fullfile(csv_dir, files(ii).name));
        if ii == 1
            time_vec = tbl.t_s;
            steps = numel(time_vec);
            positions = zeros(n, 3, steps);
        else
            if size(tbl,1) ~= steps || any(abs(tbl.t_s - time_vec) > 1e-6)
                error('Time vector mismatch in file %s', files(ii).name);
            end
        end
        positions(ii,1,:) = tbl.x_m;
        positions(ii,2,:) = tbl.y_m;
        positions(ii,3,:) = tbl.z_m;
    end
end

function rng_out = expand_range(rng_in, margin)
    span = diff(rng_in);
    if span <= 0
        span = max(abs(rng_in)); if span == 0, span = 1; end
        rng_out = rng_in(1) + span*([-0.5, 0.5]);
    else
        pad = span * margin;
        rng_out = [rng_in(1)-pad, rng_in(2)+pad];
    end
end
