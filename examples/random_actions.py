import gym
import gym_turtlebot3
import rospy

# See https://github.com/ITTcs/turtlebot3_simulations/

env_name = 'TurtleBot3_Circuit_Simple-v0'
env = gym.make(env_name)

observation = env.reset()

for _ in range(150):
    action = env.action_space.sample()
    observation, reward, done, info = env.step(action)

    if done:
        observation = env.reset()

env.close()
