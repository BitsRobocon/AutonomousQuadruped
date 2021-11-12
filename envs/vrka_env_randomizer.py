"""
CODE BASED ON EXAMPLE FROM:
@misc{coumans2017pybullet,
  title={Pybullet, a python module for physics simulation in robotics, games and machine learning},
  author={Coumans, Erwin and Bai, Yunfei},
  url={www.pybullet.org},
  year={2017},
}
Example: minitaur_env_randomizer.py
https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/env_randomizers/minitaur_env_randomizer.py
"""
import numpy as np
import env_randomizer_base

# Relative range.
vrka_BASE_MASS_ERROR_RANGE = (-0.2, 0.2)  # 0.2 means 20%
vrka_LEG_MASS_ERROR_RANGE = (-0.2, 0.2)  # 0.2 means 20%
# Absolute range.
BATTERY_VOLTAGE_RANGE = (7.0, 8.4)  # Unit: Volt
MOTOR_VISCOUS_DAMPING_RANGE = (0, 0.01)  # Unit: N*m*s/rad (torque/angular vel)
vrka_LEG_FRICTION = (0.8, 1.5)  # Unit: dimensionless


class VrkaEnvRandomizer(env_randomizer_base.EnvRandomizerBase):
    """A randomizer that change the vrka_gym_env during every reset."""
    def __init__(self,
                 vrka_base_mass_err_range=vrka_BASE_MASS_ERROR_RANGE,
                 vrka_leg_mass_err_range=vrka_LEG_MASS_ERROR_RANGE,
                 battery_voltage_range=BATTERY_VOLTAGE_RANGE,
                 motor_viscous_damping_range=MOTOR_VISCOUS_DAMPING_RANGE):
        self._vrka_base_mass_err_range = vrka_base_mass_err_range
        self._vrka_leg_mass_err_range = vrka_leg_mass_err_range
        self._battery_voltage_range = battery_voltage_range
        self._motor_viscous_damping_range = motor_viscous_damping_range

        np.random.seed(0)

    def randomize_env(self, env):
        self._randomize_vrka(env.vrka)

    def _randomize_vrka(self, vrka):
        """Randomize various physical properties of vrka.
    It randomizes the mass/inertia of the base, mass/inertia of the legs,
    friction coefficient of the feet, the battery voltage and the motor damping
    at each reset() of the environment.
    Args:
      vrka: the vrka instance in vrka_gym_env environment.
    """
        base_mass = vrka.GetBaseMassFromURDF()
        # print("BM: ", base_mass)
        randomized_base_mass = np.random.uniform(
            np.array([base_mass]) * (1.0 + self._vrka_base_mass_err_range[0]),
            np.array([base_mass]) * (1.0 + self._vrka_base_mass_err_range[1]))
        vrka.SetBaseMass(randomized_base_mass[0])

        leg_masses = vrka.GetLegMassesFromURDF()
        leg_masses_lower_bound = np.array(leg_masses) * (
            1.0 + self._vrka_leg_mass_err_range[0])
        leg_masses_upper_bound = np.array(leg_masses) * (
            1.0 + self._vrka_leg_mass_err_range[1])
        randomized_leg_masses = [
            np.random.uniform(leg_masses_lower_bound[i],
                              leg_masses_upper_bound[i])
            for i in range(len(leg_masses))
        ]
        vrka.SetLegMasses(randomized_leg_masses)

        randomized_battery_voltage = np.random.uniform(
            BATTERY_VOLTAGE_RANGE[0], BATTERY_VOLTAGE_RANGE[1])
        vrka.SetBatteryVoltage(randomized_battery_voltage)

        randomized_motor_damping = np.random.uniform(
            MOTOR_VISCOUS_DAMPING_RANGE[0], MOTOR_VISCOUS_DAMPING_RANGE[1])
        vrka.SetMotorViscousDamping(randomized_motor_damping)

        randomized_foot_friction = np.random.uniform(vrka_LEG_FRICTION[0],
                                                     vrka_LEG_FRICTION[1])
        vrka.SetFootFriction(randomized_foot_friction)