import gym

from gym_vrka.envs.vrka_bezeir_env import vrkaBezierEnv
from gym_vrka.envs.vrka_gym_env import vrkaGymEnv
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

env = gym.make('VrkaEnv-v0')
# Optional: PPO2 requires a vectorized environment to run
# the env is now wrapped automatically when passing it to the constructor
# env = DummyVecEnv([lambda: env])

model = PPO2(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=1000)
model.save("ppo2")

del model # remove to demonstrate saving and loading

model = PPO2.load("ppo2")

obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()

env.close()
