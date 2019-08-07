import gym
import gym_turtlebot3
import rospy

env_name = 'TurtleBot3_Circuit_Simple-v0'

rospy.init_node(env_name.replace('-', '_'))

env = gym.make(env_name)

observation = env.reset()

for _ in range(100):
    action = env.action_space.sample()
    observation, reward, done, info = env.step(action)

    if done:
        observation = env.reset()

env.close()
