
import math
import numpy as np

from state import State
import world as ws

from copy import copy

# Fast SLAM covariance
Q = np.diag([0.1, np.deg2rad(1.0)]) ** 2
R = np.diag([0.1, 0.2, np.deg2rad(1.0)]) ** 2

MAX_RANGE = 20.0  # maximum observation range
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
N_PARTICLE = 3  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling
dt = ws.DT*50


class Particle:
    def __init__(self, x, y, yaw, car):
        self.w = 1.0 / N_PARTICLE
        self.state = State([x, y, yaw, 0, 0, 0, 0])
        self.P = np.eye(3)
        # self.lm = np.zeros((n_landmark, LM_SIZE))
        # self.lmP = np.zeros((n_landmark * LM_SIZE, LM_SIZE))
        self.lm = []
        self.lmP = []
        self.car = car
        self.alpha = 0.0


def fast_slam2(particles, u, z, x_t=None):
    particles = predict_particles(particles, u, x_t)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles


def normalize_weight(particles):
    sum_w = sum([p.w for p in particles])

    try:
        for i in range(N_PARTICLE):
            particles[i].w /= sum_w
    except ZeroDivisionError:
        for i in range(N_PARTICLE):
            particles[i].w = 1.0 / N_PARTICLE

        return particles

    return particles


def calc_final_state(particles):
    x_est = np.zeros((STATE_SIZE, 1))

    particles = normalize_weight(particles)

    for i in range(N_PARTICLE):
        x_est[0, 0] += particles[i].w * particles[i].state.x
        x_est[1, 0] += particles[i].w * particles[i].state.y
        x_est[2, 0] += particles[i].w * particles[i].state.yaw

    return x_est


def predict_particles(particles, u, x_t):
    """
    particles: list of Particle, each with .state (VehicleState) including x, y, yaw
    control_input: np.array([[u], [v], [w]]) - shared for all particles this step
    """

    noise_std = np.array([[0.1],    # u (forward)
                        [0.3],    # v (lateral)
                        [np.deg2rad(2)]])  # w (yaw rate)
    for p in particles:

        # Add Gaussian noise to control input
        noise = np.random.randn(3, 1) * noise_std
        noisy_u = u + noise  # u is shape (3, 1)
        #print(f'without noise: {u}, with noise: {noisy_u}')

        # Create minimal state vector [x, y, yaw] as required by motion_model
        x_vec = np.array([[p.state.x],
                          [p.state.y],
                          [p.state.yaw]])

        # Predict new state using your motion model
        x_pred = motion_model(x_vec, noisy_u)

        x_err = x_pred[0, 0] - x_t.x
        y_err = x_pred[1, 0] - x_t.y
        yaw_err = x_pred[2, 0] - x_t.yaw
        x_alpha = np.clip(abs(x_err) / 0.6, 0.0, 1.0)
        y_alpha = np.clip(abs(y_err) / 0.3, 0.0, 1.0)
        yaw_alpha = np.clip(abs(yaw_err) / np.deg2rad(0.5), 0.0, 1.0)

        # Update particle state
        p.state.x = (1 - x_alpha) * x_pred[0, 0] + x_alpha * x_t.x
        p.state.y = (1 - y_alpha) * x_pred[1, 0] + y_alpha * x_t.y
        p.state.yaw = (1 - yaw_alpha) * x_pred[2, 0] + yaw_alpha * x_t.yaw

        # Optionally store u, v, w for downstream logic
        p.state.u = u[0, 0]
        p.state.v = u[1, 0]
        p.state.w = u[2, 0]

    return particles



def compute_jacobians(particle, xf, Pf, Q_cov):
    dx = xf[0, 0] - particle.state.x
    dy = xf[1, 0] - particle.state.y
    d2 = dx ** 2 + dy ** 2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.state.yaw)]).reshape(2, 1)

    Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

    Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

    Sf = Hf @ Pf @ Hf.T + Q_cov

    return zp, Hv, Hf, Sf


def update_kf_with_cholesky(xf, Pf, v, Q_cov, Hf):
    PHt = Pf @ Hf.T
    S = Hf @ PHt + Q_cov

    S = (S + S.T) * 0.5
    SChol = np.linalg.cholesky(S).T
    SCholInv = np.linalg.inv(SChol)
    W1 = PHt @ SCholInv
    W = W1 @ SCholInv.T

    x = xf + W @ v
    P = Pf - W1 @ W1.T

    return x, P



def update_landmark(particle, z, Q_cov):
    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id]).reshape(2, 1)
    Pf = particle.lmP[lm_id]

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_kf_with_cholesky(xf, Pf, dz, Q, Hf)

    particle.lm[lm_id] = xf.flatten()
    particle.lmP[lm_id] = Pf

    return particle





def compute_weight(particle, z, Q_cov):
    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id]).reshape(2, 1)
    Pf = particle.lmP[lm_id]
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        return 1.0

    num = np.exp(-0.5 * dz.T @ invS @ dz)[0, 0]
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

    w = num / den

    return w


def proposal_sampling(particle, z, Q_cov):
    lm_id = int(z[2])
    xf = particle.lm[lm_id].reshape(2, 1)
    Pf = particle.lmP[lm_id]
    x = np.array([particle.state.x, particle.state.y, particle.state.yaw]).reshape(3, 1)
    P = particle.P
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

    Sfi = np.linalg.inv(Sf)
    dz = z[0:2].reshape(2, 1) - zp
    dz[1] = pi_2_pi(dz[1])

    Pi = np.linalg.inv(P)     # RRRRRRR

    particle.P = np.linalg.inv(Hv.T @ Sfi @ Hv + Pi)
    x += particle.P @ Hv.T @ Sfi @ dz

    particle.state.x = x[0, 0]
    particle.state.y = x[1, 0]
    particle.state.yaw = x[2, 0]

    return particle



def compute_mahalanobis_distance(particle, z, lm_id, Q_cov):
    xf = particle.lm[lm_id].reshape(2, 1)
    Pf = particle.lmP[lm_id]
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)
    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])
    d2 = (dz.T @ np.linalg.inv(Sf) @ dz)[0, 0]
    return d2


def add_new_lm(particle, z, Q_cov):
    r = z[0]
    b = z[1]

    s = math.sin(particle.state.yaw + b)
    c = math.cos(particle.state.yaw + b)
    x = particle.state.x + r * c
    y = particle.state.y + r * s
    particle.lm.append(np.array([x, y]))

    dx = r * c
    dy = r * s
    d2 = dx ** 2 + dy ** 2
    d = math.sqrt(d2)
    Gz = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])
    P = np.linalg.inv(Gz) @ Q_cov @ np.linalg.inv(Gz.T)
    particle.lmP.append(P)

    return len(particle.lm) - 1



def update_with_observation(particles, z, mahal_thresh=9.8):
    for iz in range(z.shape[1]):
        obs = z[:, iz]
        for p in particles:
            best_lm_id = None
            best_d2 = float('inf')
            d2 = 0

            for lm_id, lm_pos in enumerate(p.lm):
                # Euclidean distance check
                dx = lm_pos[0] - p.state.x
                dy = lm_pos[1] - p.state.y
                dist = math.hypot(dx, dy)
                # if dist > MAX_RANGE + 1.0:
                #     continue

                d2 = compute_mahalanobis_distance(p, obs, lm_id, Q)
                if d2 < mahal_thresh and d2 < best_d2:
                    best_d2 = d2
                    best_lm_id = lm_id


            if best_lm_id is not None:
                # print(f'Match with md: {d2}')
                z_with_id = np.array([obs[0], obs[1], best_lm_id])
                w = compute_weight(p, z_with_id, Q)
                p.w *= w
                p = update_landmark(p, z_with_id, Q)
                p = proposal_sampling(p, z_with_id, Q)
            else:
                # print(obs)
                new_id = add_new_lm(p, obs, Q)
                z_with_id = np.array([obs[0], obs[1], new_id])
                p = proposal_sampling(p, z_with_id, Q)

    return particles




def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLE):
        pw.append(particles[i].w)

    pw = np.array(pw)

    n_eff = 1.0 / (pw @ pw.T)  # Effective particle number

    if n_eff < NTH:  # resampling
        w_cum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
        resample_id = base + np.random.rand(base.shape[0]) / N_PARTICLE

        indexes = []
        index = 0
        for ip in range(N_PARTICLE):
            while (index < w_cum.shape[0] - 1) \
                    and (resample_id[ip] > w_cum[index]):
                index += 1
            indexes.append(index)

        tmp_particles = particles[:]
        for i in range(len(indexes)):
            particles[i].state.x = tmp_particles[indexes[i]].state.x
            particles[i].state.y = tmp_particles[indexes[i]].state.y
            particles[i].state.yaw = tmp_particles[indexes[i]].state.yaw
            particles[i].lm = copy(tmp_particles[indexes[i]].lm)
            particles[i].lmP = copy(tmp_particles[indexes[i]].lmP)
            particles[i].w = 1.0 / N_PARTICLE

    return particles



def motion_model(x, u):
    """
    x: np.array([[x], [y], [yaw]])
    u: np.array([[u], [v], [w]])  # forward, lateral, yaw rate

    Returns updated x after DT
    """
    x_pos = x[0, 0]
    y_pos = x[1, 0]
    yaw = x[2, 0]

    u_fwd = u[0, 0]
    v_lat = u[1, 0]
    w_yaw = u[2, 0]

    # Full kinematic update (nonholonomic)
    dx = u_fwd * np.cos(yaw) - v_lat * np.sin(yaw)
    dy = u_fwd * np.sin(yaw) + v_lat * np.cos(yaw)
    dyaw = w_yaw

    x_pos += dx * dt
    y_pos += dy * dt
    yaw += dyaw * dt

    return np.array([[x_pos], [y_pos], [yaw]])


def pi_2_pi(angle):
    return angle_mod(angle)

def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle



class ControlEKF:
    def __init__(self, car, sigma_u=0.5, sigma_v=0.5, sigma_w=np.deg2rad(2)):
        self.car = car  
        self.dt = ws.DT

        # Initial control estimate: [u, v, w]
        self.x = np.zeros((3, 1))  # [u, v, w]

        # Initial covariance
        self.P = np.diag([sigma_u ** 2, sigma_v ** 2, sigma_w ** 2])

        # Process noise (uncertainty in model prediction)
        self.Q = np.diag([0.2, 0.2, np.deg2rad(1)]) ** 2

        # Measurement noise (uncertainty in sensors)
        self.R = np.diag([0.2, 0.2, np.deg2rad(1)]) ** 2

        # Measurement matrix (identity: we directly measure v, w)
        self.H = np.eye(3)

    def fuse(self, input, u_meas, v_meas, w_meas, state):
        # --- Predict step ---
        state_dot = self.car.compute_state_dot_k_fix(input, state)
        
        u_model = state_dot[3]  # u_dot → integrate for v
        v_model = state_dot[4]  # u_dot → integrate for v
        w_model = state_dot[5]  # w_dot → integrate for w

        # Predict new control (Euler integration)
        x_pred = self.x + np.array([[u_model], [v_model], [w_model]]) * self.dt
        P_pred = self.P + self.Q

        # --- Update step ---
        z = np.array([[u_meas], [v_meas], [w_meas]])
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        self.x = x_pred + K @ y
        self.P = (np.eye(3) - K @ self.H) @ P_pred

        return self.x  # returns np.array([[v], [w]])
