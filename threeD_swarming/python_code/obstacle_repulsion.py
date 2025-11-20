import numpy as np
from types import SimpleNamespace
from typing import Sequence, Union, Mapping, Any

ArrayLike = Union[np.ndarray, Sequence[Sequence[float]]]
ObsLike = Union[Mapping[str, Any], SimpleNamespace]


def obstacle_repulsion(p: ArrayLike,
                       obstacles: Sequence[ObsLike],
                       params: ObsLike) -> np.ndarray:
    """
    Compute obstacle-induced acceleration for each drone.

    Parameters
    ----------
    p : (n, 3) array-like
        Drone positions [m].
    obstacles : sequence of dicts / objects
        Each obstacle should at least have a 'type' field
        (e.g. 'cylinder', 'wall') and other fields as per case.
    params : dict / object
        Must provide default values:
            k_o   : repulsion gain
            d_safe: influence radius [m]
            fmax  : accel cap [m/s^2]

    Returns
    -------
    a_obs : (n, 3) ndarray
        Obstacle repulsion accelerations [m/s^2].
    """
    p = np.asarray(p, dtype=float)
    n = p.shape[0]
    a_obs = np.zeros((n, 3), dtype=float)

    k_o = _get_required(params, "k_o")
    d_safe = _get_required(params, "d_safe")
    fmax = _get_required(params, "fmax")

    eps = np.finfo(float).eps

    for i in range(n):
        acc_i = np.zeros(3, dtype=float)
        x, y, z = p[i, :]

        for obs in obstacles:
            gain_k = _field_with_default(obs, "k_o", k_o)
            d_safe_k = _field_with_default(obs, "d_safe", d_safe)
            fmax_k = _field_with_default(obs, "fmax", fmax)

            obs_type = _get_required(obs, "type").lower()

            if obs_type == "cylinder":
                # -------- Cylinder case --------
                cx, cy = _get_required(obs, "xy")
                R = _get_required(obs, "radius")
                z0 = _get_required(obs, "zmin")
                z1 = _get_required(obs, "zmax")

                # Vector in XY from axis center
                dx = x - cx
                dy = y - cy
                rxy = np.hypot(dx, dy)

                if rxy > 0:
                    rhat_xy = np.array([dx, dy]) / max(rxy, eps)
                else:
                    rhat_xy = np.zeros(2)

                # Candidate 1: lateral surface (clamp z to cylinder span)
                z_lat = np.clip(z, z0, z1)
                surf_x = cx + R * rhat_xy[0]
                surf_y = cy + R * rhat_xy[1]
                v_lat = np.array([x - surf_x, y - surf_y, z - z_lat])
                d_lat = np.linalg.norm(v_lat)
                if d_lat > 0:
                    n_lat = v_lat / d_lat
                else:
                    n_lat = np.zeros(3)

                # Candidate 2: top/bottom caps (closest point on disk)
                # Choose nearer cap in z
                if abs(z - z0) < abs(z - z1):
                    cap_z = z0
                else:
                    cap_z = z1

                rhat_cap = rhat_xy.copy()
                if rxy > R:
                    # Outside the disk: nearest point on the circle edge
                    cap_x = cx + R * rhat_cap[0]
                    cap_y = cy + R * rhat_cap[1]
                else:
                    # Inside disk: vertical projection to cap plane
                    cap_x = x
                    cap_y = y

                v_cap = np.array([x - cap_x, y - cap_y, z - cap_z])
                d_cap = np.linalg.norm(v_cap)
                if d_cap > 0:
                    n_cap = v_cap / d_cap
                else:
                    n_cap = np.zeros(3)

                # Choose the closer surface
                if d_lat <= d_cap:
                    d = d_lat
                    n_hat = n_lat
                else:
                    d = d_cap
                    n_hat = n_cap

                # Short-range repulsion, zero beyond d_safe_k
                if d < d_safe_k and np.any(n_hat):
                    # f(d) = gain * (1/d - 1/d_safe), capped at fmax
                    f = gain_k * (1.0 / d - 1.0 / d_safe_k)
                    f = max(0.0, min(f, fmax_k))
                    acc_i += f * n_hat

            elif obs_type == "wall":
                # -------- Wall case --------
                frame = _build_wall_frame(obs)

                rel = p[i, :] - frame.center
                local = np.array([
                    np.dot(rel, frame.u),
                    np.dot(rel, frame.v),
                    np.dot(rel, frame.n),
                ])

                clamped = np.array([
                    _clamp_component(local[0], frame.half_w),
                    _clamp_component(local[1], frame.half_h),
                    _clamp_component(local[2], frame.half_t),
                ])

                closest = (frame.center
                           + clamped[0] * frame.u
                           + clamped[1] * frame.v
                           + clamped[2] * frame.n)

                vec_to_surface = p[i, :] - closest
                d = np.linalg.norm(vec_to_surface)

                if d < 1e-9:
                    n_hat_surface, d = _wall_escape_direction(local, frame)
                else:
                    n_hat_surface = vec_to_surface / d

                if d < d_safe_k and np.any(n_hat_surface):
                    f = gain_k * (1.0 / d - 1.0 / d_safe_k)
                    f = max(0.0, min(f, fmax_k))
                    acc_i += f * n_hat_surface

            else:
                # Extend: spheres, boxes, polygons, etc.
                pass

        a_obs[i, :] = acc_i

    return a_obs


# ---------------------------------------------------------------------
# Helper functions (Python equivalents of the MATLAB subfunctions)
# ---------------------------------------------------------------------

def _pick_orthogonal(n_hat: np.ndarray) -> np.ndarray:
    """
    Return a vector roughly orthogonal to n_hat (not necessarily unit).
    """
    n_hat = np.asarray(n_hat, dtype=float)
    idx = np.argmin(np.abs(n_hat))  # index of smallest component
    basis = np.zeros(3)
    basis[idx] = 1.0
    v = np.cross(n_hat, basis)
    if np.linalg.norm(v) < 1e-8:
        # Try another axis
        basis = np.zeros(3)
        new_idx = (idx + 1) % 3
        basis[new_idx] = 1.0
        v = np.cross(n_hat, basis)
    return v


def _sign_nonzero(val: float) -> int:
    """
    Return sign, treating zero as +1 to push outward consistently.
    """
    return 1 if val >= 0 else -1


def _clamp_component(x: float, limit: float) -> float:
    return float(np.clip(x, -limit, limit))


def _build_wall_frame(obs: ObsLike) -> SimpleNamespace:
    """
    Construct orthonormal basis for a rectangular wall obstacle.

    Expected fields on obs:
        center   : (3,) array-like
        normal   : (3,) array-like
        width    : float
        height   : float
        thickness: float (optional; default 1.0)
        up       : (3,) array-like (optional)
    """
    center = np.asarray(_get_required(obs, "center"), dtype=float).ravel()
    n_hat = np.asarray(_get_required(obs, "normal"), dtype=float).ravel()
    n_hat = n_hat / max(np.linalg.norm(n_hat), 1e-9)

    up_vec_opt = _get_optional(obs, "up", None)
    if up_vec_opt is not None and len(up_vec_opt) != 0:
        up_vec = np.asarray(up_vec_opt, dtype=float).ravel()
    else:
        up_vec = _pick_orthogonal(n_hat)

    # Remove any normal component from up_vec to keep axes orthogonal
    up_vec = up_vec - np.dot(up_vec, n_hat) * n_hat
    if np.linalg.norm(up_vec) < 1e-8:
        up_vec = _pick_orthogonal(n_hat)
    up_vec = up_vec / max(np.linalg.norm(up_vec), 1e-9)

    u_hat = np.cross(n_hat, up_vec)
    if np.linalg.norm(u_hat) < 1e-8:
        up_vec = _pick_orthogonal(n_hat)
        u_hat = np.cross(n_hat, up_vec)
    u_hat = u_hat / max(np.linalg.norm(u_hat), 1e-9)

    v_hat = np.cross(n_hat, u_hat)
    v_hat = v_hat / max(np.linalg.norm(v_hat), 1e-9)

    width = float(_get_required(obs, "width"))
    height = float(_get_required(obs, "height"))
    thickness = float(_get_optional(obs, "thickness", 1.0))

    return SimpleNamespace(
        center=center,
        n=n_hat,
        u=u_hat,
        v=v_hat,
        half_w=width / 2.0,
        half_h=height / 2.0,
        half_t=thickness / 2.0,
    )


def _wall_escape_direction(local: np.ndarray,
                           frame: SimpleNamespace) -> tuple[np.ndarray, float]:
    """
    Handle points sitting inside the wall volume (penetration).

    local : (3,) array in wall's local coordinates
    """
    local = np.asarray(local, dtype=float).ravel()
    gaps = np.array([
        frame.half_w - abs(local[0]),
        frame.half_h - abs(local[1]),
        frame.half_t - abs(local[2]),
    ])

    idx = int(np.argmin(gaps))  # 0, 1, or 2
    dist = max(float(gaps[idx]), 1e-3)

    dirs = np.zeros(3)
    dirs[idx] = _sign_nonzero(local[idx])

    if idx == 0:
        n_hat_surface = dirs[idx] * frame.u
    elif idx == 1:
        n_hat_surface = dirs[idx] * frame.v
    else:
        n_hat_surface = dirs[idx] * frame.n

    n_hat_surface = n_hat_surface / max(np.linalg.norm(n_hat_surface), 1e-9)
    return n_hat_surface, dist


def _field_with_default(obs: ObsLike, name: str, default_val: Any) -> Any:
    """
    Equivalent of MATLAB field_with_default: obs.(name) if present & non-empty,
    otherwise default_val.

    Works for both dict-like and attribute-like objects.
    """
    if isinstance(obs, Mapping):
        val = obs.get(name, None)
    else:
        val = getattr(obs, name, None)
    if val is None:
        return default_val
    # Treat empty list/array like "empty" in MATLAB
    if isinstance(val, (list, tuple, np.ndarray)) and len(val) == 0:
        return default_val
    return val


def _get_required(obj: Any, name: str) -> Any:
    """
    Get a required field from dict or object; raise KeyError/AttributeError
    if missing.
    """
    if isinstance(obj, Mapping):
        if name not in obj:
            raise KeyError(f"Required field '{name}' missing in obstacle/params")
        return obj[name]
    if not hasattr(obj, name):
        raise AttributeError(f"Required attribute '{name}' missing in obstacle/params")
    return getattr(obj, name)


def _get_optional(obj: Any, name: str, default: Any = None) -> Any:
    """
    Get an optional field from dict or object, with default.
    """
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)
