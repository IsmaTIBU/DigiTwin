from main_analyse import main_analyse
from Tests.plotly3d import *


def menu():
    print("Menu:")
    print("1. Matrices, Direct Kinematics, Inverse Kinematics")
    print("2. Visualize Temporal Motion Laws and Associated Graphs")
    print("0. Exit")

    choice = input("Please choose an option: ")
    return choice


if __name__ == "__main__":
    while True:
        choice = menu()
        if choice == "1":
            main_analyse()
        if choice == "2":
            V1 = float(input("Speed 1:\n"))
            V2 = float(input("Speed 2:\n"))
            x = float(input("Shared x-coordinate for A and B:\n"))
            A = np.array([x, float(input("y-coordinate for A:\n")), float(input("z-coordinate for A:\n"))])
            B = np.array([x, float(input("y-coordinate for B:\n")), float(input("z-coordinate for B:\n"))])
            K = float(input("Acceleration:\n"))
            while K <= 0.0:
                print("Acceleration must be positive! Please enter a positive acceleration.")
                K = float(input("Acceleration:\n"))
            traj(A, B, V1, V2, K, Debug=True)
            print("Would you like to run a simulation with these data?")
            k = input("1: Yes 2: No\n")
            if k == "1":
                bras_rob_model3D_animation(A, B, V1, V2, K)
