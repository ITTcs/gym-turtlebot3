from baselines.common import plot_util as pu
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

folder_log = 'TurtleBot3_Circuit_Simple_Continuous-v0_1e4_ddpg'

results = pu.load_results('./logs/' + folder_log)
pu.plot_results(results, average_group=True, split_fn=lambda _: '', shaded_std=False)
plt.show()