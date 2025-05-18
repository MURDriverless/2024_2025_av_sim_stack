import math
import numpy as np
import matplotlib.pyplot as plt

import world as ws
from state import State
from gpu import GPU


GRAVITY = 9.81
MAX_KIN = 1.5
MIN_DYN = 3

def ccw(A, B, C):                                                               # Check if three points are listed in a counterclockwise order
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def segments_intersect(A, B, C, D):                                             # Return True if line segment AB intersects with CD
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


class Vehicle:
    def __init__(self, world):
        self.world = world



class Car(Vehicle):                                                             # My version of Paul's Kinematic + Dynamic model with rk4

    def __init__(self, world, track, car_name, state = State()):
        
        if world not in ws.worlds:
            raise ValueError(f"World: {world.name} doesnt exist")
        
        if track not in world.tracks:
            raise ValueError(f"Track: {track.name} doesnt exist in World: {world.name}")

        self.name = car_name
        self.world = world
        self.track = track
        track.add_car(self)

        if state.x == 0:
            self.state = State([self.track.start_point[0], self.track.start_point[1], state.yaw, state.u, state.v, state.w, state.delta])
        else:
            self.state = State([state.x, state.y, state.yaw, state.u, state.v, state.w, state.delta])
        self.input = 0

        self.lidar = False
        self.camera = False
        self.controller = False
        self.gpu = GPU(self)

        self.mass = 1420
        self.lf = 1
        self.lr = 1.9
        self.Iz = 1500
        self.Cm = 0.15
        self.Cd = 0.1
        self.max_steer = np.deg2rad(60.0)

        self.Bp = 10
        self.Cp = 1.9
        self.Dp = 1
        self.Ep = 0.97

        self.visible_range = 8
        self.visible_angle = np.deg2rad(85.0)

        self.visible_cones = []
        self.forward_cones = []
        self.forward_cones_lidar = []
        self.new_cone = 0

        self.seen_cones = set()
        self.seen_valid_edges = set()
        self.seen_invalid_edges = set()
        self.valid_edges = []
        self.invalid_edges = []

        self.path = []
        self.seen_path = set()

        self.state_var = None

        self.lap_start = False
        self.lap_finish = False

        self.i = 0


    def get_state(self):
        return self.state.get_state()

    # F = (Fcmd/100.0) * Cm - Cd * vx^2
    # alpha_f = atan( (vy + lf omega) / vx ) - delta
    # alpha_r = atan( (vy - lf omega) / vx )

    # Fyf = Fzf Dp sin( C atan( B alpha_f - E()))
    # Fyr = Fzr Dp sin( C atan( B alpha_r - E()))

    # > F_lateral = Fz · D · sin(C · arctan(B·slip – E · (B·slip – arctan(B·slip))))    

    # px_dot    = vx * cos(theta) - vy * sin(theta)
    # py_dot    = vx * sin(theta) + vy * cos(theta)
    # theta_dot = omega

    # vx_dot    = (1/m)  * (F   - Fyf sin(delta) + m v_y omega )
    # vy_dot    = (1/m)  * (Fyr + Fyf cos(delta) - m v_x omega )
    # omega_dot = (1/Iz) * (-Fyr lr + Fyf lf cos(delta))

    def compute_state_dot_k(self, F, state):

        F_on_m = F / self.mass
        tan_delta = np.tan(state.delta)
        delta_dot_sec2_delta_vx_plus_tan_delta_v_dot = self.input.delta_dot*(1.0+tan_delta*tan_delta)*state.u + tan_delta*F_on_m

        x_dot = state.u * np.cos(state.yaw) - state.v * np.sin(state.yaw) 
        y_dot = state.u * np.sin(state.yaw) + state.v * np.cos(state.yaw) 
        yaw_dot = state.w
        u_dot = F_on_m
        v_dot = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * self.lr / (self.lf+self.lr)
        w_dot = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * 1.0      / (self.lf+self.lr) 
        delta_dot = self.input.delta_dot

        state_dot = np.array([x_dot, y_dot, yaw_dot, u_dot, v_dot, w_dot, delta_dot])

        return state_dot
    
    def compute_state_dot_k_fix(self, input, state):

        F = input.F * self.Cm - self.Cd * state.u * state.u * np.sign(state.u)     # Compute the Forward Force
        F_on_m = F / self.mass
        tan_delta = np.tan(state.delta)
        delta_dot_sec2_delta_vx_plus_tan_delta_v_dot = input.delta_dot*(1.0+tan_delta*tan_delta)*state.u + tan_delta*F_on_m

        x_dot = state.u * np.cos(state.yaw) - state.v * np.sin(state.yaw) 
        y_dot = state.u * np.sin(state.yaw) + state.v * np.cos(state.yaw) 
        yaw_dot = state.w
        u_dot = F_on_m
        v_dot = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * self.lr / (self.lf+self.lr)
        w_dot = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * 1.0      / (self.lf+self.lr) 
        delta_dot = input.delta_dot

        state_dot = np.array([x_dot, y_dot, yaw_dot, u_dot, v_dot, w_dot, delta_dot])

        return state_dot

    def compute_state_dot_d(self, F, state):

        alpha_f = -np.arctan( (state.v + self.lf * state.w) / state.u) + state.delta        # Compute the front wheel slip angle
        alpha_r = -np.arctan( (state.v - self.lr * state.w) / state.u)                      # Compute the rear wheel slip angle
        Fzf = GRAVITY * self.mass * self.lr / (self.lf + self.lr)                           # Compute the front wheel normal force
        Fzr = GRAVITY * self.mass * self.lf / (self.lf + self.lr)                           # Compute the rear wheel normal force
        Fyf = Fzf * self.Dp * np.sin( self.Cp * np.arctan( self.Bp * alpha_f - self.Ep * ( self.Bp * alpha_f - np.arctan(self.Bp * alpha_f))))      # Compute the front wheel lateral force
        Fyr = Fzr * self.Dp * np.sin( self.Cp * np.arctan( self.Bp * alpha_r - self.Ep * ( self.Bp * alpha_r - np.arctan(self.Bp * alpha_r))))      # Compute the rear wheel lateral force

        x_dot = state.u * np.cos(state.yaw) - state.v * np.sin(state.yaw)
        y_dot = state.u * np.sin(state.yaw) + state.v * np.cos(state.yaw)
        yaw_dot = state.w
        u_dot = (1/self.mass)  * (F - Fyf * np.sin(state.delta) ) + state.v * state.w
        v_dot = (1/self.mass)  * (Fyr + Fyf * np.cos(state.delta) ) - state.u * state.w
        w_dot = (1/self.Iz) * (-Fyr * self.lr + Fyf * self.lf * np.cos(state.delta))
        delta_dot = self.input.delta_dot

        state_dot = np.array([x_dot, y_dot, yaw_dot, u_dot, v_dot, w_dot, delta_dot])

        return state_dot

    def compute_state_dot(self, state):

        F = self.input.F * self.Cm - self.Cd * state.u * state.u * np.sign(state.u)     # Compute the Forward Force
        mix_factor = min(max((state.u-MAX_KIN)/(MIN_DYN-MAX_KIN),0),1)

        state_dot = (1-mix_factor)*self.compute_state_dot_k(F, state) + mix_factor*self.compute_state_dot_d(F, state)

        return state_dot



    def integrate(self):

        m1 = self.compute_state_dot(self.state)                     # Compute the eom for the RK4 sequence of states
        m2 = self.compute_state_dot(self.state + 0.5*m1*ws.DT)
        m3 = self.compute_state_dot(self.state + 0.5*m2*ws.DT)
        m4 = self.compute_state_dot(self.state + m3*ws.DT)

        self.state_var = (1/6) * (m1 + 2*m2 + 2*m3 + m4) * ws.DT         # Compute the state update
        state_plus = self.state + self.state_var

        if segments_intersect(self.get_position(), [state_plus.x, state_plus.y], self.track.start_line[0].get_position(), self.track.start_line[1].get_position()):
            if not self.lap_start:
                print('START')
                self.lap_start = True
            else:
                print('FINISH')
                self.lap_finish = True

        return state_plus
    


    def add_lidar(self, lidar):

        lidar.car = self
        self.gpu.lidar = lidar
        self.gpu.update()



    def add_camera(self, camera):

        camera.car = self
        self.gpu.camera = camera
        self.gpu.update()


    def add_controller(self, controller):
        
        controller.wheelbase = self.lf + self.lr
        controller.max_delta = self.max_steer
        controller.car = self
        self.gpu.controller = controller
        self.gpu.update()


    def start_controller(self):

        self.gpu.start = True
        self.gpu.update()



    def get_position(self):
        return [self.state.x, self.state.y]
    


    def update(self, input):

        self.input = input
        self.state.delta = np.clip(self.state.delta, -self.max_steer, self.max_steer)

        if self.state.delta >= self.max_steer and self.input.delta_dot > 0:
            self.input.delta_dot = 0
        elif self.state.delta <= -self.max_steer and self.input.delta_dot < 0:
            self.input.delta_dot = 0


        state_plus = self.integrate()
        self.state = state_plus

