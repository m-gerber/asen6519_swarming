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
        switch obs.type
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
                if d < d_safe && any(n_hat)
                    % Smooth barrier that grows rapidly near the surface
                    % f(d) = k_o * (1/d - 1/d_safe), capped at fmax
                    f = k_o * (1/d - 1/d_safe);         % [m/s^2]
                    f = max(0, min(f, fmax));
                    acc_i = acc_i + f * n_hat;
                end

            otherwise
                % Extend: spheres, boxes, polygons, etc.
        end
    end
    a_obs(i,:) = acc_i;
end
end
