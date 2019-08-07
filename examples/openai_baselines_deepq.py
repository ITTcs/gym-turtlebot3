import gym
import gym_turtlebot3
import rospy
import baselines.run as run
import os

env_name = 'TurtleBot3_Circuit_Simple-v0'
alg = 'deepq'
num_timesteps = '1e4'

name_ref = env_name + '_' + num_timesteps + '_' + alg

my_args = [
    '--alg=' + alg, 
    '--env=' + env_name, 
    '--save_path=./models/' + name_ref + '.pkl',
    '--num_timesteps=' + num_timesteps]

os.environ["OPENAI_LOG_FORMAT"] = "csv"
os.environ["OPENAI_LOGDIR"] = './logs/' + name_ref

rospy.init_node(env_name.replace('-', '_'))

run.main(my_args)
