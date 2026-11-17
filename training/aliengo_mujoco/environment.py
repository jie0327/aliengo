"""Small experimental AlienGo task; not a calibrated deployment environment."""
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np


class AliengoEnv(gym.Env):
    metadata = {}

    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.base = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        if self.model.nq != 19 or self.model.nv != 18 or self.model.nu != 12 or self.base < 0:
            raise ValueError('Unexpected floating-base model or actuator count')
        self.data = mujoco.MjData(self.model)
        self.neutral = np.array([-.05, .7, -1.4, .05, .7, -1.4,
                                 -.05, .7, -1.4, .05, .7, -1.4])
        self.action_space = spaces.Box(-1.0, 1.0, (12,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (46,), dtype=np.float32)
        self.previous = np.zeros(12)
        self.count = 0

    def observe(self):
        rotation = self.data.xmat[self.base].reshape(3, 3)
        body_velocity = rotation.T @ self.data.qvel[:3]
        gravity = rotation.T @ np.array([0., 0., -1.])
        # MuJoCo free-joint angular qvel is expressed in the local body frame.
        angular_velocity = self.data.qvel[3:6]
        values = np.concatenate((body_velocity, angular_velocity, gravity,
                                 self.data.qpos[7:] - self.neutral,
                                 self.data.qvel[6:] * .05, self.previous, [.2]))
        return values.astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = [0, 0, .4, 1, 0, 0, 0]
        self.data.qpos[7:] = self.neutral + self.np_random.uniform(-.01, .01, 12)
        self.previous[:] = 0
        self.count = 0
        mujoco.mj_forward(self.model, self.data)
        return self.observe(), {}

    def step(self, action):
        action = np.clip(np.asarray(action), -1, 1)
        start_x = self.data.qpos[0]
        target = self.neutral + .25 * action
        for _ in range(10):
            # Experimental position-PD action, capped by the model actuators.
            self.data.ctrl[:] = 120 * (target - self.data.qpos[7:]) - 5 * self.data.qvel[6:]
            mujoco.mj_step(self.model, self.data)
        self.count += 1
        upright = float(self.data.xmat[self.base].reshape(3, 3)[2, 2])
        velocity = float((self.data.qpos[0] - start_x) / .02)
        reward = (1.5 * np.exp(-((velocity - .2) / .25) ** 2) + .5 * upright
                  - .003 * np.sum(action ** 2) - .005 * np.sum((action - self.previous) ** 2)
                  - .05 * (self.data.qvel[3] ** 2 + self.data.qvel[4] ** 2))
        self.previous = action.copy()
        finite = bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
        terminated = bool(not finite or self.data.qpos[2] < .2 or self.data.qpos[2] > .6 or upright < .55)
        truncated = self.count >= 500
        info = {'x': float(self.data.qpos[0]), 'z': float(self.data.qpos[2]),
                'upright': upright, 'velocity': velocity}
        return self.observe(), float(reward), terminated, truncated, info
