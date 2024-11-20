import numpy as np
import itertools
import matplotlib.pyplot as plt
import Dynamic_model_3teth_revC


def main():
    teth1_range = np.linspace(-1.0/12, 1.0/12, 5)
    teth2_range = np.linspace(-1.0/12, 1.0/12, 5)
    teth3_range = np.linspace(-1.0/12, 1.0/12, 5)

    in_error = list(itertools.product(teth1_range, teth2_range, teth3_range))

    ang_err = np.zeros(len(in_error))
    force_err = np.zeros(len(in_error))

    plt.style.use('default')
    fig1 = plt.figure(figsize=(10, 8))
    ax1 = fig1.add_subplot(111, projection='3d')

    for i in range(len(in_error)):
        ang_err_vec, force_err_vec = Dynamic_model_3teth_revC.main(in_error[i])
        ang_err[i] = max(ang_err_vec)
        force_err[i] = max(force_err_vec)

        if (ang_err[i] < 2.0) & (force_err[i] < 5.0):
            ax1.scatter(in_error[i][0], in_error[i][1], in_error[i][2], s=10, c="g")
        else:
            ax1.scatter(in_error[i][0], in_error[i][1], in_error[i][2], s=10, c="r")



    plt.show()


    # print(ang_err)
    # print(force_err)














if __name__ == "__main__":
    main()