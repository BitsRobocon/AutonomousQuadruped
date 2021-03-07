import gym
from src.quadruped.quadruped.RL.gymVrka.gym_vrka.envs.vrka_bezeir_env  import vrkaBezierEnv

env = gym.make('VrkaEnv-v1')
env.reset()
for _ in range(1000):
    env.render()
    env.step(env.action_space.sample()) # take a random action
env.close()