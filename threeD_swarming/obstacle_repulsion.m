function a_obs = obstacle_repulsion(p, obstacles, params)
% p: (n x 3) drone positions [m]
% obstacles: struct array
% params.k_o   repulsion gain
% params.d_safe influence radius [m]
% params.fmax   accel cap [m/s^2]

n = size(p,1);
a_obs = zeros(n,3);
k_o    = params.k_o;
d_safe = params.d_safe;
fmax   = params.fmax;

for i = 1:n
    acc_i = [0 0 0];
    for k = 1:numel(obstacles)
        obs = obstacles(k);
        gain_k   = field_with_default(obs, 'k_o', k_o);
        d_safe_k = field_with_default(obs, 'd_safe', d_safe);
        fmax_k   = field_with_default(obs, 'fmax', fmax);
        switch lower(obs.type)
            case 'cylinder'
                % Unpack
                cx = obs.xy(1); cy = obs.xy(2);
                R  = obs.radius;
                z0 = obs.zmin; z1 = obs.zmax;

                x = p(i,1); y = p(i,2); z = p(i,3);

                % Vector in XY from axis center
                dx = x - cx; dy = y - cy;
                rxy = hypot(dx, dy);                 % radial distance to axis
                rhat_xy = (rxy > 0) * [dx dy] / max(rxy, eps); % unit in XY

                % Candidate 1: lateral surface (clamp z to cylinder span)
                z_lat  = min(max(z, z0), z1);
                surf_x = cx + R * rhat_xy(1);
                surf_y = cy + R * rhat_xy(2);
                v_lat  = [x - surf_x, y - surf_y, z - z_lat];
                d_lat  = norm(v_lat);
                if d_lat > 0, n_lat = v_lat / d_lat; else, n_lat = [0 0 0]; end

                % Candidate 2: top/bottom caps (closest point on disk)
                % Pick which cap is nearer in z
                cap_z  = (abs(z - z0) < abs(z - z1)) * z0 + (abs(z - z0) >= abs(z - z1)) * z1;
                rhat_cap = rhat_xy;   % same direction in XY
                if rxy > R
                    % Outside the disk: nearest point on the circle edge
                    cap_x = cx + R * rhat_cap(1);
                    cap_y = cy + R * rhat_cap(2);
                else
                    % Inside the disk: vertical projection to cap plane
                    cap_x = x; cap_y = y;
                end
                v_cap = [x - cap_x, y - cap_y, z - cap_z];
                d_cap = norm(v_cap);
                if d_cap > 0, n_cap = v_cap / d_cap; else, n_cap = [0 0 0]; end

                % Choose the closer surface
                if d_lat <= d_cap
                    d = d_lat; n_hat = n_lat;
                else
                    d = d_cap; n_hat = n_cap;
                end

                % Short-range repulsion, zero beyond d_safe
                if d < d_safe_k && any(n_hat)
                    % Smooth barrier that grows rapidly near the surface
                    % f(d) = gain * (1/d - 1/d_safe), capped at fmax
                    f = gain_k * (1/d - 1/d_safe_k);     % [m/s^2]
                    f = max(0, min(f, fmax_k));
                    acc_i = acc_i + f * n_hat;
                end

            case 'wall'
                frame = build_wall_frame(obs);
                rel = p(i,:) - frame.center;
                local = [dot(rel, frame.u), dot(rel, frame.v), dot(rel, frame.n)];
                clamped = [clamp_component(local(1), frame.half_w), ...
                           clamp_component(local(2), frame.half_h), ...
                           clamp_component(local(3), frame.half_t)];
                closest = frame.center + clamped(1)*frame.u + clamped(2)*frame.v + clamped(3)*frame.n;
                vec_to_surface = p(i,:) - closest;
                d = norm(vec_to_surface);

                if d < 1e-9
                    [n_hat_surface, d] = wall_escape_direction(local, frame);
                else
                    n_hat_surface = vec_to_surface / d;
                end

                if d < d_safe_k && any(n_hat_surface)
                    f = gain_k * (1/d - 1/d_safe_k);
                    f = max(0, min(f, fmax_k));
                    acc_i = acc_i + f * n_hat_surface;
                end

            otherwise
                % Extend: spheres, boxes, polygons, etc.
        end
    end
    a_obs(i,:) = acc_i;
end
end

function v = pick_orthogonal(n_hat)
% Return a unit-length vector roughly orthogonal to n_hat (fallback basis)
    [~, idx] = min(abs(n_hat));
    basis = zeros(1,3);
    basis(idx) = 1;
    v = cross(n_hat, basis);
    if norm(v) < 1e-8
        basis = zeros(1,3);
        basis(mod(idx,3)+1) = 1;
        v = cross(n_hat, basis);
    end
end

function s = sign_nonzero(val)
% Return sign, treating zero as +1 to push outward consistently
    if val >= 0
        s = 1;
    else
        s = -1;
    end
end

function val = clamp_component(x, limit)
    val = max(-limit, min(limit, x));
end

function frame = build_wall_frame(obs)
% Construct orthonormal basis for a rectangular wall obstacle
    center = obs.center(:)';
    n_hat = obs.normal(:)';
    n_hat = n_hat / norm(n_hat);

    if isfield(obs,'up') && ~isempty(obs.up)
        up_vec = obs.up(:)';
    else
        up_vec = pick_orthogonal(n_hat);
    end
    % remove any normal component from up_vec to keep axes orthogonal
    up_vec = up_vec - dot(up_vec, n_hat) * n_hat;
    if norm(up_vec) < 1e-8
        up_vec = pick_orthogonal(n_hat);
    end
    up_vec = up_vec / max(norm(up_vec), 1e-9);
    u_hat = cross(n_hat, up_vec);
    if norm(u_hat) < 1e-8
        up_vec = pick_orthogonal(n_hat);
        u_hat = cross(n_hat, up_vec);
    end
    u_hat = u_hat / norm(u_hat);
    v_hat = cross(n_hat, u_hat);
    v_hat = v_hat / norm(v_hat);

    frame.center = center;
    frame.n = n_hat;
    frame.u = u_hat;
    frame.v = v_hat;
    frame.half_w = obs.width / 2;
    frame.half_h = obs.height / 2;
    if isfield(obs,'thickness') && ~isempty(obs.thickness)
        frame.half_t = obs.thickness / 2;
    else
        frame.half_t = 1.0; % fallback thickness
    end
end

function [n_hat_surface, dist] = wall_escape_direction(local, frame)
% Handle points sitting inside the wall volume (penetration)
    gaps = [frame.half_w - abs(local(1)), ...
            frame.half_h - abs(local(2)), ...
            frame.half_t - abs(local(3))];
    [dist, idx] = min(gaps);
    dist = max(dist, 1e-3);
    dirs = zeros(1,3);
    dirs(idx) = sign_nonzero(local(idx));
    if idx == 1
        n_hat_surface = dirs(idx) * frame.u;
    elseif idx == 2
        n_hat_surface = dirs(idx) * frame.v;
    else
        n_hat_surface = dirs(idx) * frame.n;
    end
    n_hat_surface = n_hat_surface / max(norm(n_hat_surface), 1e-9);
end

function val = field_with_default(obs, name, default_val)
    if isfield(obs, name) && ~isempty(obs.(name))
        val = obs.(name);
    else
        val = default_val;
    end
end
