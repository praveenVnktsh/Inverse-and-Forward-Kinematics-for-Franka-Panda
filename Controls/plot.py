import matplotlib.pyplot as plt
import numpy as np


force = np.loadtxt('force_vs_time_dynamic_force.csv',delimiter=',')
print(force.shape)
plt.plot(force[:, 0], force[:, 1])
force = np.loadtxt('force_vs_time_imp_force.csv',delimiter=',')
plt.plot(force[:, 0], force[:, 1])
plt.xlabel('time (s)')
plt.ylabel('Force (N)')
plt.title("Force vs Time")
plt.grid()
plt.legend(['Force Control', 'Impedance Control'])
plt.savefig('force_control_dynamic.png')
plt.show()