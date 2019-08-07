import gym
import gym_turtlebot3
import numpy as np
import rospy

from stable_baselines.sac.policies import FeedForwardPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import SAC

env_name = 'TurtleBot3_Circuit_Simple-v0'

# Custom MLP policy of two layers of size 128 each
class CustomSACPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomSACPolicy, self).__init__(*args, **kwargs,
                                           layers=[128, 128],
                                           layer_norm=False,
                                           feature_extraction="mlp")

env = gym.make(env_name)
env = DummyVecEnv([lambda: env])

rospy.init_node(env_name.replace('-', '_'))

model = SAC(CustomSACPolicy, env, verbose=1)
model.learn(total_timesteps=10000, log_interval=10)
model.save(env_name)
