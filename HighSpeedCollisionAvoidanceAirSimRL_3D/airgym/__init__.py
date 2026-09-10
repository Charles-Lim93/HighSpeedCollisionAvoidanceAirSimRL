from gym.envs.registration import register

# Spatial-3D (y-z) drone collision-avoidance environment.
# Action = [v_y, v_z] (continuous, 2-D); forward speed is held constant.
# Named "vertical" on purpose: it extends the lateral (1-D) proposed
# algorithm with a vertical axis.
register(
    id="airsim-drone-vertical-v0",
    entry_point="airgym.envs:AirSimDroneVerticalEnv",
)
