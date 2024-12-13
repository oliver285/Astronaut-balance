import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import Parameters_3_Tether_Dynamic_Model

def calculate_tether_vecs(COM, teth_anchor, offset):
    # determine the tether unit vector
    teth1_vec = np.array(teth_anchor[0][:]) - (np.array(COM) - np.array(offset[0][:]))
    teth2_vec = np.array(teth_anchor[1][:]) - (np.array(COM) - np.array(offset[1][:]))
    teth3_vec = np.array(teth_anchor[2][:]) - (np.array(COM) - np.array(offset[2][:]))
    teth1_hat = teth1_vec/np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec/np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec/np.linalg.norm(teth3_vec)
    lengths = np.array([np.linalg.norm(teth1_vec),np.linalg.norm(teth2_vec),np.linalg.norm(teth3_vec)])

    return (teth1_hat, teth2_hat, teth3_hat, lengths)


def plot_tether_init_setup(teth_anchor, offset, COM, r):
    plt.style.use('default')
    figsize = (10, 8)

    hip_anchor = np.array(COM) - np.array(offset)
    teth_vecs = np.array(teth_anchor) - hip_anchor
    # tether vectors
    vecs1 = np.array([np.hstack((teth_anchor[0][:], -teth_vecs[0][:])),
                     np.hstack((teth_anchor[1][:], -teth_vecs[1][:])),
                     np.hstack((teth_anchor[2][:], -teth_vecs[2][:]))])  # tether vecs
    # offset vectors
    vecs2 = np.array([np.hstack((hip_anchor[0][:], offset[0][:])),
                     np.hstack((hip_anchor[1][:], offset[1][:])),
                     np.hstack((hip_anchor[2][:], offset[2][:]))])  # offset vecs

    # circle for waist
    theta = np.linspace(0, 2 * np.pi, 100)  # Angle values

    # Circle coordinates
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.full_like(theta, COM[2])  # Constant z

    X1, Y1, Z1, U1, V1, W1 = zip(*vecs1)
    X2, Y2, Z2, U2, V2, W2 = zip(*vecs2)

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(X1, Y1, Z1, U1, V1, W1, pivot='tail', arrow_length_ratio=0.1, color="red", label="tether vectors")
    ax.quiver(X2, Y2, Z2, U2, V2, W2, pivot='tail', arrow_length_ratio=0.4, color="blue", label="offset vectors")
    ax.plot(x, y, z, color="black", label="waist approximation")
    ax.set_xlabel('X (ft)')
    ax.set_ylabel('Y (ft)')
    ax.set_zlabel('Z (ft)')
    ax.set_title('tether visual')
    # ax.set_xlim([-3, 3])
    # ax.set_ylim([-3, 3])
    # ax.set_zlim([-3, 3])
    ax.legend()
    ax.invert_zaxis()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig('plots/3D_visual.png')


def main():
    COM = [0, 0, -2.5]
    p = Parameters_3_Tether_Dynamic_Model.Parameters()
    teth_anchor = p.teth_anchor
    offset = p.offset
    r = p.r

    plot_tether_init_setup(teth_anchor, offset, COM, r)
    plt.show()

if __name__ == "__main__":
    main()