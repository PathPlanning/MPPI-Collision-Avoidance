



import numpy as np


class BaseDyn:
    def dynamic_model(self, state, control, dt):
        pass

    def state_shift(self, state, dt):
        pass


class DiffDriveDyn(BaseDyn):

    def dynamic_model(self, state, control, dt):
        dt = np.float128(dt)
        theta = np.float128(state[2])
        v = np.float128(control[0])
        w = np.float128(control[1])
        dx = v * np.cos(theta, dtype=np.float128) * dt
        dy = v * np.sin(theta, dtype=np.float128) * dt
        dtheta = w * dt
        return np.array([dx, dy, dtheta])

    def state_shift(self, state, dt):
        return state


class CarLikeDyn (BaseDyn):

    def __init__(self, dist_l):
        self.dist_between_axles = dist_l

    def dynamic_model(self, state, control, dt):
        inv_l = np.float128(1.0) / np.float128(self.dist_between_axles)
        dt = np.float128(dt)
        theta = np.float128(state[2])
        v = np.float128(control[0])
        phi = np.float128(control[1])
        dx = v * np.cos(theta, dtype=np.float128) * dt
        dy = v * np.sin(theta, dtype=np.float128) * dt
        dtheta = (v * inv_l * np.tan(phi, dtype=np.float128)) * dt

        return np.array([dx, dy, dtheta])

    def state_shift(self, state, dt):
        return state
