from gym.envs.registration import register

register(
    id="VrkaEnv-v0",
    entry_point='src.quadruped.quadruped.RL.gym-vrka.gym_vrka.envs:vrkaGymEnv',
    max_episode_steps=1000,
)

register(
    id="VrkaEnv-v1",
    entry_point='src.quadruped.quadruped.RL.gym-vrka.gym_vrka.envs:vrkaBezierEnv',
    max_episode_steps=1000,
)