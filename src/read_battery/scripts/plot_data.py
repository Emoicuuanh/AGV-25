#!/usr/bin/env python3

import matplotlib.pyplot as plt
import sys
import rospy
from shutil import copyfile, copy

folderPath = "../results/"
fileTime = ""


def read_data(path, num_row):
    cnt = 0
    ret = [[] for _ in range(num_row)]
    with open(path, "r") as f:
        while True:
            line = f.readline()

            if line.endswith("\n"):
                line = line[:-1]
            print(cnt)
            print(not line)
            if cnt == 0:
                fileTime = line
            if cnt > 1:
                if (not line) or ("STOP" in line):
                    break

                data = line.split(" ")
                if num_row > len(data):
                    print(
                        "Error! num_row: {}, len {}".format(num_row, len(data))
                    )
                    break

                for i in range(num_row):
                    ret[i].append(float(data[i]))

            cnt += 1
    return fileTime, ret


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "--help" or sys.argv[1] == "-H":
            print(
                """(1) ./show_plt.py: Ve do thi cho ket qua moi nhat `lastest*`\n(2) ./show_plt.py <time>: Ve do thi cho ket qua cua file log co time tuong ung"""
            )
            exit()
        else:
            fileTime, plan = read_data(
                folderPath + sys.argv[1] + "_plan.txt", 2
            )
            fileTime, log = read_data(folderPath + sys.argv[1] + "_log.txt", 7)
    else:
        rospy.logwarn("debug")
        # fileTime, plan = read_data(folderPath + "lastest_plan.txt", 2)
        fileTime, log = read_data(folderPath + "lastest_log.txt", 7)

    plt.figure(0)
    fig, ax = plt.subplots(6)

    ax[0].plot(log[0], log[1], label="vol")
    ax[0].set_ylim(0, 55)
    ax[0].legend()

    ax[1].plot(log[0], log[2], label="current")
    ax[1].set_ylim(0, 15)
    ax[1].legend()

    ax[2].plot(log[0], log[3], label="capacity_remain")
    ax[2].set_ylim(0, 60)
    ax[2].legend()

    ax[3].plot(log[0], log[4], label="percentage")
    ax[3].set_ylim(0, 100)
    ax[3].legend()

    ax[4].plot(log[0], log[5], label="temperature")
    ax[4].set_ylim(0, 50)
    ax[4].legend()

    ax[5].plot(log[0], log[6], label="baterry_state")
    ax[5].set_ylim(0, 5)
    ax[5].legend()

    plt.grid(True)
    plt.legend()
    # plt.show()
    plt.savefig(folderPath + "battery.png")
    plt.close()

    # plt.figure(1)
    # # plt.plot(plan[0], plan[1], label="MkacPlan")
    # # plt.plot(log[5], log[6], alpha=0.5, label="odom position")
    # plt.scatter(log[5], log[6], s=1, marker="o", color="g", label="odom pos")
    # # plt.scatter(plan[0], plan[1], s=1, marker="o", color='r', label="MkacPlan")
    # plt.grid(True)
    # plt.legend()
    # # plt.show()
    # plt.savefig(folderPath + "position.png")
    # copyfile(folderPath + "vel.png", folderPath + fileTime + "_vel.png")
    copyfile(folderPath + "battery.png", folderPath + fileTime + "_battery.png")
