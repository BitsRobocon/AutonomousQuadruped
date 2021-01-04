from gym.envs.registration import register

register(
    id="SpotMicroEnv-v0",
    entry_point='spotmicro.spot_gym_env:spotGymEnv',
    max_episode_steps=1000,
)