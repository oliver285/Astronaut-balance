### in python
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def equations(p, a, b, c):
    x, y, z = p
    return (
        (x - 2)**2 + (y)**2 + z**2 - a**2,
        (x + 2)**2 + (y)**2 + z**2 - b**2,
        x**2 + (y - 2*np.sqrt(3))**2 + z**2 - c**2
    )

def calculate_apex(a, b, c):
    initial_guess = [2, 2, 4]  # Start with a point above the base
    apex = fsolve(equations, initial_guess, args=(a, b, c))
    return apex

def plot_tetrahedron(apex):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Base vertices
    base = np.array([
        [2, 0, 0],
        [-2, 0, 0],
        [0, 2*np.sqrt(3), 0]
    ])

    # Plot base
    for i in range(3):
        j = (i + 1) % 3
        ax.plot([base[i,0], base[j,0]], [base[i,1], base[j,1]], [base[i,2], base[j,2]], 'b-')

    # Plot edges to apex
    for i in range(3):
        ax.plot([base[i,0], apex[0]], [base[i,1], apex[1]], [base[i,2], apex[2]], 'r-')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Tetrahedron Visualization')

    # Set equal aspect ratio
    max_range = np.array([apex.max()-apex.min(), base[:,1].max()-base[:,1].min(), apex[2]-base[:,2].min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(apex.max()+apex.min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(base[:,1].max()+base[:,1].min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(apex[2]+base[:,2].min())
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')

    plt.show()

def main():
    print("Enter the lengths of the edges connecting the apex to the base:")
    a = float(input("Length AD: "))
    b = float(input("Length BD: "))
    c = float(input("Length CD: "))


    print(a)
    apex = calculate_apex(a, b, c)
    print(f"Apex coordinates: ({apex[0]:.5f}, {apex[1]:.5f}, {apex[2]:.5f})")

    plot_tetrahedron(apex)

if __name__ == "__main__":
    main()
