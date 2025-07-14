import matplotlib.pyplot as plt

def create_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 3)
    ax.set_zlim(0, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.ion()
    plt.show()
    return fig, ax

def update_plot(ax, X, Y, Z):
    ax.cla()
    ax.plot(X, Y, Z, '-o', color='blue')
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 3)
    ax.set_zlim(0, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.draw()
    plt.pause(0.05)
