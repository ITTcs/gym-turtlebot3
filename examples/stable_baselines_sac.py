import gym
import gym_turtlebot3
import rospy

from stable_baselines.sac.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import SAC

env_name = 'TurtleBot3_Circuit_Simple_Continuous-v0'

rospy.init_node(env_name.replace('-', '_'))

env = gym.make(env_name)
env = DummyVecEnv([lambda: env])

model = SAC(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=50000, log_interval=10)
model.save(env_name)
