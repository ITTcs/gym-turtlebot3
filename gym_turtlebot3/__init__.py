from gym.envs.registration import register
import math

register(
    id='TurtleBot3-v0',
    entry_point='gym_turtlebot3.envs:TurtleBot3Env'
)

goal_list = [   [0.505, 2.005],
                [1.005, 2.505],
                [2.005, 2.505],
                [2.505, 2.005],
                [2.505, 1.005],
                [2.005, 0.505],
                [1.005, 0.505],
                [0.505, 1.005]] 

max_env_size = math.hypot(3, 3)

register(
    id='TurtleBot3_Circuit_Simple-v0',
    entry_point='gym_turtlebot3.envs:TurtleBot3Env',
    kwargs={'goal_list': goal_list, 'max_env_size': max_env_size}
)

register(
    id='TurtleBot3_Circuit_Simple_Continuous-v0',
    entry_point='gym_turtlebot3.envs:TurtleBot3Env',
    kwargs={'goal_list': goal_list, 'max_env_size': max_env_size, 'continuous': True}
)