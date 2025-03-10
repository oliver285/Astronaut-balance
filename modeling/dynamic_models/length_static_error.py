# this requires ffmpeg is installed
# use this install guide: https://phoenixnap.com/kb/ffmpeg-windows


import numpy as np
import itertools
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from modeling.dynamic_models.model_archive import Dynamic_model_3teth_revC
from tqdm import tqdm
matplotlib.rcParams['animation.ffmpeg_path'] = r'C:\Users\Peter\OneDrive\Desktop\ffmpeg\ffmpeg-2024-11-18-git-970d57988d-full_build\bin\ffmpeg.exe'


def create_animated_scatter():
    # Generate data
    teth1_range = np.linspace(-1.0 / 12, 1.0 / 12, 50)
    teth2_range = np.linspace(-1.0 / 12, 1.0 / 12, 50)
    teth3_range = np.linspace(-1.0 / 12, 1.0 / 12, 50 )
    in_error = list(itertools.product(teth1_range, teth2_range, teth3_range))

    # Calculate errors (assuming Dynamic_model_3teth_revC.main is available)
    points = np.array(in_error)
    colors = []
    for point in tqdm(in_error):
        ang_err_vec, force_err_vec = Dynamic_model_3teth_revC.main(point)
        if (max(ang_err_vec) < 2.0) and (max(force_err_vec) < 5.0):
            colors.append('g')
        else:
            colors.append('r')

    # Setup the figure with a specific DPI for video export
    plt.style.use('default')
    fig = plt.figure(figsize=(10, 8), dpi=100)
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
                                   interval=100,  # 500ms between frames
                                   blit=False,
                                   repeat=True)

    # Set up the FFmpeg writer
    writer = animation.FFMpegWriter(
        fps=2,  # 2 FPS since interval is 500ms
        metadata=dict(artist='Me'),
        bitrate=2000
    )

    # Save the animation
    output_file = 'plots/scatter_animation.mp4'
    anim.save(output_file, writer=writer)
    print(f"Animation saved as {output_file}")

    return anim


def main():
    anim = create_animated_scatter()
    # Keep a reference to prevent garbage collection
    plt.show()


if __name__ == "__main__":
    main()