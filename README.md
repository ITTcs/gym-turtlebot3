# TurtleBot3 Gym env
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

This is a gym env to work with the TurtleBot3 gazebo simulations, allowing the use of [OpenAI Baselines](https://github.com/openai/baselines) and [Stable Baselines](https://github.com/hill-a/stable-baselines) deep reinforcement learning algorithms in the robot navigation training. See the `examples` folder to check some Python programs.

This project is a part of the development of some gazebo environments to apply deep-rl algorithms.

[Research Gazebo environments for TurtleBot3 robot](https://github.com/ITTcs/turtlebot3_simulations)

## Installation

```
git clone https://github.com/ITTcs/gym-turtlebot3
cd ITTcs/gym-turtlebot3
pip install -e .
```

This is an example using the OpenAI Baselines `DDPG` algorithm in a TurtleBot3 environment.

`.\examples\openai_baselines_ddpg.py`:

```python
import gym
import gym_turtlebot3
import rospy
import baselines.run as run
import os

env_name = 'TurtleBot3_Circuit_Simple_Continuous-v0'
alg = 'ddpg'
num_timesteps = '1e4'

name_ref = env_name + '_' + num_timesteps + '_' + alg

my_args = [
    '--alg=' + alg, 
    '--env=' + env_name, 
    '--num_timesteps=' + num_timesteps]

os.environ["OPENAI_LOG_FORMAT"] = "csv"
os.environ["OPENAI_LOGDIR"] = './logs/' + name_ref

rospy.init_node(env_name.replace('-', '_'))

run.main(my_args)
```

Before running a Python program you first need run the specified environment. To the `TurtleBot3_Circuit_Simple_Continuous-v0` env for example, would be:

```
roslaunch turtlebot3_gazebo turtlebot3_circuit_simple.launch
```

These resouces are part of a master's thesis where we use the Python [fuzzylab](https://github.com/ITTcs/fuzzylab) library to create fuzzy logic controllers with the implementation of deep reinforcement learning algorithms.

To cite this repository in publications:

    @misc{gymturtlebot3,
      author = {Avelar, Eduardo},
      title = {gym-turtlebot3},
      year = {2019},
      publisher = {GitHub},
      journal = {GitHub repository},
      howpublished = {\url{https://github.com/ITTcs/gym-turtlebot3}},
    }

