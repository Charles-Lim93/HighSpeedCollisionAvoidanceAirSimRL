from gym.envs.registration import register

register(
    id="airsim-drone-sample-v0", entry_point="airgym.envs:AirSimDroneEnv",
)

register(
    id="airsim-drone-sample-continuous-v0", entry_point = "airgym.envs:AirSimDroneContinuousEnv",
)

register(
    id="airsim-drone-sample-continuous-1d-v0", entry_point = "airgym.envs:AirSimDroneContinuous1dEnv",
)


register(
    id="airsim-drone-continuous-image-v0", entry_point = "airgym.envs:AirSimDroneContinuousImageOnlyEnv",
)

# # Register AirSim environment as a gym environment
# register(
#     id="airsim-drone-sample-continuous-image-only-v0", entry_point="airgym.envs:TestEnv",
# )



register(
    id="airsim-drone-sample-discrete-v0", entry_point = "airgym.envs:AirSimDroneDiscreteEnv",
)

register(
    id="airsim-drone-sample-discrete-image-only-v0", entry_point = "airgym.envs:AirSimDroneDiscreteImageOnlyEnv",
)


register(
    id="airsim-car-sample-v0", entry_point="airgym.envs:AirSimCarEnv",
)


