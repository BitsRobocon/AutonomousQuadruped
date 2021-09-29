from gym.envs.registration import register

register(
    id="VrkaEnv-v0",
    entry_point='gym_vrka.envs.vrka_gym_env:vrkaGymEnv',
    max_episode_steps=1000,
)

register(
    id="VrkaEnv-v1",
    entry_point='gym_vrka.envs.vrka_bezeir_env:vrkaBezierEnv',
    max_episode_steps=1000,
)