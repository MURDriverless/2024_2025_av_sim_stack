import numpy as np
import cvxpy as cp
import math
import scipy.linalg as la

import world as ws
from state import Input
from state import State_C

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]


def angle_mod(x, zero_2_2pi=False, degree=False):

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


def pi_2_pi(angle):
    return angle_mod(angle)

def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.u) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


class Controller:
    def __init__(self):
        self.car = None
        self.time = 0.0

    def set_car(self, car):
        self.car = car

    def set_path(self, path):
        raise NotImplementedError("Subclasses should implement this!")

    def pursue(self):
        """
        Core control logic, should return control inputs (e.g., throttle and steering).
        """
        raise NotImplementedError("Subclasses must implement pursue()")






class PurePursuit(Controller):
    def __init__(self, lookahead_distance, u_max, k_speed_c, k_throttle):
        """
        Initialize the pure pursuit controller.

        Parameters:
        - lookahead_distance: Distance ahead to target (L_d)
        - wheelbase: Distance between front and rear axle (L)
        - dt: Time step (s)
        - max_delta_dot: Optional max rate of change for steering (rad/s)
        """
        #super().__init__()
        self.car = None

        self.lookahead_distance = lookahead_distance
        self.wheelbase = 0
        self.dt = ws.DT
        self.max_delta_dot = np.deg2rad(50)
        self.path = None
        self.prev_target = None
        self.target = None
        self.target_dist = 0
        self.prev_delta = 0.0  # Previous steering angle
        self.u_max = u_max            # Max speed (m/s)
        self.k_speed_c = k_speed_c        # Gain for curvature-based slowdown
        self.k_throttle = k_throttle
        

        self.time = 0.0


    def set_path(self, path):
        self.path = np.array(path)


    def find_target_point(self):

        max_dist = 0
        target_point = self.prev_target

        for point in self.path:

            distance = np.linalg.norm(point)

            # if distance == 0:
            #     continue

            # Get unit vector from car to point
            # direction_to_point = point / distance
            # heading_vector = np.array([1.0, 0.0])

            # Compute angle between heading and direction to point
            # dot_product = np.clip(np.dot(heading_vector, direction_to_point), -1.0, 1.0)
            # angle_deg = np.degrees(np.arccos(dot_product))

            # Check if within FOV (±125°) and within lookahead distance
            # if angle_deg <= 85 and distance <= self.lookahead_distance and distance > max_dist:
            if distance <= self.lookahead_distance and distance > max_dist:
                max_dist = distance
                target_point = point
                self.target_dist = max_dist

        self.prev_target = target_point
        return target_point


    def compute_steering_rate(self):
        target = self.find_target_point()
        
        self.target = target
        local_x = target[0]
        local_y = target[1]

        if local_x == 0:
            return 0.0

        L2 = local_x**2 + local_y**2
        if L2 == 0:
            curvature = 0.0
        else:
            curvature = (2 * local_y) / L2

        self.curvature = curvature
        desired_delta = np.arctan(self.wheelbase * self.curvature)

        delta_dot = (desired_delta - self.car.state.delta) / self.dt

        if self.max_delta_dot is not None:
            delta_dot = np.clip(delta_dot, -self.max_delta_dot, self.max_delta_dot)

        return delta_dot
    
    def compute_velocity(self):
        """
        Return desired velocity based on curvature.
        """
        return (self.u_max) / (1 + self.k_speed_c * abs(self.curvature))


    def compute_throttle_force(self, current_u, desired_u):
        """
        Simple proportional controller to compute throttle force input F.
        """
        return self.k_throttle * (desired_u - current_u)
    

    def update(self, current_speed):

        delta_dot = self.compute_steering_rate()
        velocity = self.compute_velocity()
        F = self.compute_throttle_force(current_speed, velocity)

        self.time += self.dt
        return Input(F, delta_dot)
    



class LQR(Controller):
    def __init__(self):
        super().__init__()
        self.car = None
        self.time = 0.0

    def set_car(self, car):
        self.car = car

    def set_path(self, path):
        raise NotImplementedError("Subclasses should implement this!")

    def pursue(self):
        """
        Core control logic, should return control inputs (e.g., throttle and steering).
        """
        raise NotImplementedError("Subclasses must implement pursue()")
    
    def solve_dare(A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        x = Q
        x_next = Q
        max_iter = 150
        eps = 0.01

        for i in range(max_iter):
            x_next = A.T @ x @ A - A.T @ x @ B @ \
                    la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
            if (abs(x_next - x)).max() < eps:
                break
            x = x_next

        return x_next
    
    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eig_result = la.eig(A - B @ K)

        return K, X, eig_result[0]



class MPC(Controller):
    def __init__(self, car, u_max, horizon=5):
        """
        Initialize the MPC controller.

        Parameters:
        - car: The car object that the controller will control.
        - horizon: Prediction horizon (number of time steps)
        """
        super().__init__()
        self.car = car  # Store the car object
        self.horizon = horizon
        self.dt = ws.DT
        self.path = None
        self.u_max = u_max

        # MPC weights
        self.Q = np.diag([1.0, 1.0, 0.5, 0.1])  # State error weights
        self.R = np.diag([0.1, 0.01])            # Control input weights

        # Constraints
        self.max_throttle = 2000.0  # Example max force
        self.max_delta_dot = np.deg2rad(50)  # Max steering rate

    def set_path(self, path):
        """
        Set the reference path.
        Expects a list/array of (x, y) midpoints.
        """
        self.path = np.array(path)

    def plan_mpc(self):
        """
        Solve the MPC optimization to find optimal control inputs.
        """

        # Current state
        x0 = State_C(([
            self.car.state.x,
            self.car.state.y,
            self.car.state.yaw,
            self.car.state.u
        ]))

        # Generate reference trajectory over horizon
        cx, cy, cyaw, sp, dl = self.generate_reference()
        ref_traj, _, _ = calc_ref_trajectory(x0, cx, cy, cyaw, None, sp, dl, 0)

        # Optimization variables
        x = cp.Variable((4, self.horizon + 1))  # [x, y, yaw, u]
        u = cp.Variable((2, self.horizon))      # [throttle_force, delta_dot]

        cost = 0
        constraints = [x[:,0] == x0]

        for t in range(self.horizon):
            # Cost for deviation from reference
            cost += cp.quad_form(x[:,t] - ref_traj[:,t], self.Q)
            # Cost for control effort
            cost += cp.quad_form(u[:,t], self.R)

            # Dynamics constraints (simple model: kinematic or very simple dynamic)
            next_x = self.vehicle_model(x[:,t], u[:,t])
            constraints += [x[:,t+1] == next_x]

            # Input constraints
            constraints += [
                cp.abs(u[0,t]) <= self.max_throttle,
                cp.abs(u[1,t]) <= self.max_delta_dot
            ]

        # Terminal cost
        cost += cp.quad_form(x[:,self.horizon] - ref_traj[:,-1], self.Q)

        # Solve
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)

        # Return first optimal control input
        if prob.status not in ["optimal", "optimal_inaccurate"]:
            print("Warning: MPC solve failed, using zero inputs")
            return np.zeros(2)

        return u.value[:,0]

    def generate_reference(self):
        """
        Generate cx, cy, cyaw, sp, dl from the path.
        """
        cx = self.path[:, 0]
        cy = self.path[:, 1]

        cyaw = np.arctan2(np.diff(cy, append=cy[-1]), np.diff(cx, append=cx[-1]))
        sp = np.ones_like(cx) * self.u_max  # Assume constant target speed
        dl = np.mean(np.hypot(np.diff(cx, append=cx[-1]), np.diff(cy, append=cy[-1])))  # average path intervals

        return cx, cy, cyaw, sp, dl

    def vehicle_model(self, state, control):
        """
        Simple discrete vehicle model for prediction.
        state = [x, y, yaw, u]
        control = [F, delta_dot]
        """

        x, y, yaw, u = state
        F, delta_dot = control

        m = self.car.mass  # Mass
        L = self.car.lf + self.car.lr  # Wheelbase

        u_dot = F / m
        yaw_rate = (u / L) * np.tan(self.car.state.delta)

        # Discrete update
        x_next = x + u * np.cos(yaw) * self.dt
        y_next = y + u * np.sin(yaw) * self.dt
        yaw_next = yaw + yaw_rate * self.dt
        u_next = u + u_dot * self.dt

        return cp.hstack([x_next, y_next, yaw_next, u_next])

    def update(self):
        """
        Main control update: Solve MPC and return control inputs.
        """
        opt_u = self.plan_mpc()
        F, delta_dot = opt_u
        self.car.update(Input(F, delta_dot))  # Pass control inputs to car
        return Input(F, delta_dot)








# class MPC(Controller):
#     def __init__(self, horizon=5):
#         """
#         Initialize the MPC controller.

#         Parameters:
#         - horizon: Prediction horizon (number of time steps)
#         """
#         super().__init__()
#         self.horizon = horizon
#         self.dt = ws.DT
#         self.path = None

#         # MPC weights
#         self.Q = np.diag([1.0, 1.0, 0.5, 0.1])  # State error weights
#         self.R = np.diag([0.1, 0.01])            # Control input weights

#         # Constraints
#         self.max_throttle = 2000.0  # Example max force
#         self.max_delta_dot = np.deg2rad(50)  # Max steering rate

#     def set_path(self, path):
#         """
#         Set the reference path.
#         Expects a list/array of (x, y) midpoints.
#         """
#         self.path = np.array(path)

#     def plan_mpc(self):
#         """
#         Solve the MPC optimization to find optimal control inputs.
#         """

#         # Current state
#         x0 = np.array([
#             self.car.state.x,
#             self.car.state.y,
#             self.car.state.yaw,
#             self.car.state.u
#         ])

#         # Generate reference trajectory over horizon
#         cx, cy, cyaw, sp, dl = self.generate_reference()
#         ref_traj, _ = calc_ref_trajectory(x0, cx, cy, cyaw, sp, dl, 0)

#         # Optimization variables
#         x = cp.Variable((4, self.horizon + 1))  # [x, y, yaw, u]
#         u = cp.Variable((2, self.horizon))      # [throttle_force, delta_dot]

#         cost = 0
#         constraints = [x[:,0] == x0]

#         for t in range(self.horizon):
#             # Cost for deviation from reference
#             cost += cp.quad_form(x[:,t] - ref_traj[:,t], self.Q)
#             # Cost for control effort
#             cost += cp.quad_form(u[:,t], self.R)

#             # Dynamics constraints (simple model: kinematic or very simple dynamic)
#             next_x = self.vehicle_model(x[:,t], u[:,t])
#             constraints += [x[:,t+1] == next_x]

#             # Input constraints
#             constraints += [
#                 cp.abs(u[0,t]) <= self.max_throttle,
#                 cp.abs(u[1,t]) <= self.max_delta_dot
#             ]

#         # Terminal cost
#         cost += cp.quad_form(x[:,self.horizon] - ref_traj[:,-1], self.Q)

#         # Solve
#         prob = cp.Problem(cp.Minimize(cost), constraints)
#         prob.solve(solver=cp.OSQP)

#         # Return first optimal control input
#         if prob.status not in ["optimal", "optimal_inaccurate"]:
#             print("Warning: MPC solve failed, using zero inputs")
#             return np.zeros(2)

#         return u.value[:,0]

#     def generate_reference(self):
#         """
#         Generate cx, cy, cyaw, sp, dl from the path.
#         """
#         cx = self.path[:, 0]
#         cy = self.path[:, 1]

#         cyaw = np.arctan2(np.diff(cy, append=cy[-1]), np.diff(cx, append=cx[-1]))
#         sp = np.ones_like(cx) * self.car.u_max  # Assume constant target speed
#         dl = np.hypot(np.diff(cx, append=cx[-1]), np.diff(cy, append=cy[-1]))  # Path intervals

#         return cx, cy, cyaw, sp, dl

#     def vehicle_model(self, state, control):
#         """
#         Simple discrete vehicle model for prediction.
#         state = [x, y, yaw, u]
#         control = [F, delta_dot]
#         """

#         x, y, yaw, u = state
#         F, delta_dot = control

#         m = self.car.m  # Mass
#         L = self.car.L  # Wheelbase

#         u_dot = F / m
#         yaw_rate = (u / L) * np.tan(self.car.state.delta)

#         # Discrete update
#         x_next = x + u * np.cos(yaw) * self.dt
#         y_next = y + u * np.sin(yaw) * self.dt
#         yaw_next = yaw + yaw_rate * self.dt
#         u_next = u + u_dot * self.dt

#         return cp.hstack([x_next, y_next, yaw_next, u_next])

#     def update(self):
#         """
#         Main control update: Solve MPC and return control inputs.
#         """
#         opt_u = self.plan_mpc()
#         F, delta_dot = opt_u
#         self.time += self.dt
#         return Input(F, delta_dot)