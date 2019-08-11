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

rospy.init_node(env_name.replace('-', '_'))

env = gym.make(env_name)
env = DummyVecEnv([lambda: env])

model = SAC(CustomSACPolicy, env, verbose=1)
model.learn(total_timesteps=int(1e4), log_interval=10)
model.save(env_name)
