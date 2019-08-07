import gym
import gym_turtlebot3
import rospy
import baselines.run as run
import os

env_name = 'TurtleBot3_Circuit_Simple-v0'
alg = 'deepq'
model_name = 'TurtleBot3_Circuit_Simple-v0_1e4_deepq.pkl'

rospy.init_node(env_name.replace('-', '_'))

my_args = [
    '--alg=' + alg,  
    '--env=' + env_name, 
    '--load_path=./models/' + model_name,
    '--num_timesteps=0',
    '--play']

run.main(my_args)
    