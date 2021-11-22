from pickle import TRUE
import os
import numpy as np
import gym
from gui import GUI
from vrka_bezeir_env import vrkaBezierEnv
from vrka_gym_env import vrkaGymEnv
from bezier import BezierGait
from openloopcontroller import BezierStepper
from vrka_env_randomizer import VrkaEnvRandomizer 
#from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
from ars import ARSAgent,Normalizer,Policy
from Vrkakinematics import VrkaModel
import matplotlib as plt
import argparse

# ARGUMENTS
descr = "Spot Mini Mini ARS Agent Evaluator."
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-hf",
                    "--HeightField",
                    help="Use HeightField",
                    action='store_true')
parser.add_argument("-ta",
                    "--TrueAction",
                    help="Plot Action as seen by the Robot.",
                    action='store_true')
#env = gym.make('VrkaEnv-v0')
# Optional: PPO2 requires a vectorized environment to run
# the env is now wrapped automatically when passing it to the constructor
# env = DummyVecEnv([lambda: env])
ARGS = parser.parse_args()

if ARGS.HeightField:
    height_field = True
else:
    height_field = False
env_randomizer = VrkaEnvRandomizer()
env = vrkaBezierEnv(render=True,
                    on_rack=False,
                    height_field=True,
                    draw_foot_path=False,
                    contacts=True,
                    env_randomizer=env_randomizer)

# Set seeds
seed = 0
env.seed(seed)
np.random.seed(seed)

state_dim = env.observation_space.shape[0]
print("STATE DIM: {}".format(state_dim))
action_dim = env.action_space.shape[0]
print("ACTION DIM: {}".format(action_dim))
max_action = float(env.action_space.high[0])

env.reset()

spot = VrkaModel()

bz_step = BezierStepper(dt=0.01)
bzg = BezierGait(dt=0.01)

# Initialize Normalizer
normalizer = Normalizer(state_dim)

# Initialize Policy
policy = Policy(state_dim, action_dim)

if ARGS.HeightField:
    height_field = True
else:
    height_field = False
# to GUI or not to GUI
gui = True
max_timesteps = 4e6
# Initialize Agent with normalizer, policy and gym env
my_path = os.path.abspath(os.path.dirname(__file__))
results_path = os.path.join(my_path, "../results")
models_path = os.path.join(my_path,"")
agent = ARSAgent(normalizer, policy, env, bz_step, bzg, spot, gui)
# agent_num = 0
# if ARGS.AgentNum:
#     agent_num = ARGS.AgentNum
# if os.path.exists(models_path + "/" + file_name + str(agent_num) +
#                     "_policy"):
print("Loading Existing agent")
agent.load(models_path + "/" +'vrka_ars_1069')
agent.policy.episode_steps = np.inf
policy = agent.policy

env.reset()
episode_reward = 0
episode_timesteps = 0
episode_num = 0

print("STARTED MINITAUR TEST SCRIPT")

t = 0
while t < (int(max_timesteps)):

    episode_reward, episode_timesteps = agent.deployTG()

    t += episode_timesteps
    # episode_reward = agent.train()
    # +1 to account for 0 indexing.
    # +0 on ep_timesteps since it will increment +1 even if done=True
    print("Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
        t, episode_num, episode_timesteps, episode_reward))
    episode_num += 1

    # # Plot Policy Output
    # if ARGS.PlotPolicy or ARGS.TrueAction or ARGS.SaveData:
    #     if ARGS.TrueAction:
    #         action_name = "robot_act"
    #         action = np.array(agent.true_action_history)
    #     else:
    #         action_name = "agent_act"
    #         action = np.array(agent.action_history)

    #     if ARGS.SaveData:
    #         if height_field:
    #             terrain_name = "rough_"
    #         else:
    #             terrain_name = "flat_"
    #         np.save(
    #                 results_path + "/" + "policy_out_" + terrain_name + action_name, action)

    #         print("SAVED DATA")
    # if ARGS.PlotPolicy or ARGS.TrueAction or ARGS.SaveData:
    if ARGS.TrueAction:
        action_name = "robot_act"
        action = np.array(agent.true_action_history)
    else:
        action_name = "agent_act"
        action = np.array(agent.action_history)
    ClearHeight_act = action[:, 0]
    BodyHeight_act = action[:, 1]
    Residuals_act = action[:, 2:]

    plt.plot(ClearHeight_act,
                label='Clearance Height Mod',
                color='green')
    plt.plot(BodyHeight_act,
                label='Body Height Mod',
                color='darkviolet')

    # FL
    plt.plot(Residuals_act[:, 0],
                label='Residual: FL (x)',
                color='limegreen')
    plt.plot(Residuals_act[:, 1],
                label='Residual: FL (y)',
                color='lime')
    plt.plot(Residuals_act[:, 2],
                label='Residual: FL (z)',
                color='green')

    # FR
    plt.plot(Residuals_act[:, 3],
                label='Residual: FR (x)',
                color='lightskyblue')
    plt.plot(Residuals_act[:, 4],
                label='Residual: FR (y)',
                color='dodgerblue')
    plt.plot(Residuals_act[:, 5],
                label='Residual: FR (z)',
                color='blue')

    # BL
    plt.plot(Residuals_act[:, 6],
                label='Residual: BL (x)',
                color='firebrick')
    plt.plot(Residuals_act[:, 7],
                label='Residual: BL (y)',
                color='crimson')
    plt.plot(Residuals_act[:, 8],
                label='Residual: BL (z)',
                color='red')

    # BR
    plt.plot(Residuals_act[:, 9],
                label='Residual: BR (x)',
                color='gold')
    plt.plot(Residuals_act[:, 10],
                label='Residual: BR (y)',
                color='orange')
    plt.plot(Residuals_act[:, 11],
                label='Residual: BR (z)',
                color='coral')

    plt.xlabel("Epoch Iteration")
    plt.ylabel("Action Value")
    plt.title("Policy Output")
    plt.legend()
    plt.show()

env.close()










# model = ARSAgent('MlpPolicy', env)
# model.learn(total_timesteps=1000)

# model.save("ars")

# del model # remove to demonstrate saving and loading

# model = ARSAgent.load("ars")

# obs = env.reset()
# for i in range(1000):
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

