import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import  style

fig = plt.figure(1)
plt1 = fig.add_subplot(1,1,1)

def live(data):
    data = open('error.txt','r').read()
    value = data.split(',')
    plt1.clear()
    plt1.set_title('End-Effector Pose Error')
    plt1.set_xlabel('Poses')
    plt1.set_ylabel('Error')
    plt.grid(True)

    plt1.plot(value)

ani = animation.FuncAnimation(fig, live)
plt.show()