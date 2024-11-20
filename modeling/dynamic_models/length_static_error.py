import numpy as np
import itertools
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import Dynamic_model_3teth_revC
import time


def create_animated_scatter():
    # Generate data
    teth1_range = np.linspace(-1.0/12, 1.0/12, 5)
    teth2_range = np.linspace(-1.0/12, 1.0/12, 5)
    teth3_range = np.linspace(-1.0/12, 1.0/12, 5)
    in_error = list(itertools.product(teth1_range, teth2_range, teth3_range))
    
    # Calculate errors (assuming Dynamic_model_3teth_revC.main is available)
    points = np.array(in_error)
    colors = []
    for point in in_error:
        ang_err_vec, force_err_vec = Dynamic_model_3teth_revC.main(point)
        if (max(ang_err_vec) < 2.0) and (max(force_err_vec) < 5.0):
            colors.append('g')
        else:
            colors.append('r')
    
    # Setup the figure
    plt.style.use('default')
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set axis labels
    ax.set_xlabel('Tether 1')
    ax.set_ylabel('Tether 2')
    ax.set_zlabel('Tether 3')
    
    # Get unique x values and sort them
    x_values = sorted(list(set(points[:, 0])))
    
    # Initialize empty scatter plot
    scat = ax.scatter([], [], [], s=10)
    
    def init():
        ax.view_init(elev=20, azim=45)
        return scat,
    
    def update(frame):
        ax.cla()  # Clear the current axes
        
        # Set labels and limits
        ax.set_xlabel('Tether 1')
        ax.set_ylabel('Tether 2')
        ax.set_zlabel('Tether 3')
        ax.set_xlim([min(teth1_range), max(teth1_range)])
        ax.set_ylim([min(teth2_range), max(teth2_range)])
        ax.set_zlim([min(teth3_range), max(teth3_range)])
        
        # Calculate current x_value index considering the animation loop
        x_index = frame % len(x_values)
        
        # Plot all points up to and including the current x-value
        current_x = x_values[x_index]
        mask = points[:, 0] <= current_x
        
        # Plot points
        ax.scatter(points[mask, 0], points[mask, 1], points[mask, 2],
                  c=[colors[i] for i in range(len(colors)) if mask[i]],
                  s=10)
        
        # Add title showing progress
        ax.set_title(f'Building visualization: Layer {x_index + 1}/{len(x_values)}')
        
        return scat,
    
    # Create animation
    anim = animation.FuncAnimation(fig, update, init_func=init,
                                 frames=len(x_values) * 2,  # Multiply by 2 to show two complete cycles
                                 interval=500,  # 500ms between frames
                                 blit=False,
                                 repeat=True)
    
    plt.show()
    return anim


def main():
    start = time.time()
    anim = create_animated_scatter()
    end = time.time()
    total_time = end - start
    print(total_time)
    # Keep a reference to prevent garbage collection
    plt.show()


if __name__ == "__main__":
    main()
