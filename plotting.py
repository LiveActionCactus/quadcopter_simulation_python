# Creates and visual animation of quadcopter state information
#
# By: Patrick Ledzian
# Date: 18 Apr 2020

"""
Creates and plays an animation of a 13-state quadcopter simulation. Works along with the quadcopter class descriptor
found in quadcopter.py.
"""

# External Libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
import time

# Project Libraries
import quadcopter

# TODO: set the plotting sample rate in code, need to think about this a bit (falls in category with pos/att asynchronicity)
# TODO: maybe have this class inherit from Quadcopter() to get it's properties, is that necessary? Currently using Quadcopter() static methods
# TODO: do I need a quaternion class to encapsulate it's functionality? (maybe when I add the non-linear controller?) could have a bunch of static methods
# TODO: add pre-plotting of waypoints
# TODO: add trajectory ghost line to track in gray (see how close we get), have error metric plotting (maybe as part of analysis class?)
# TODO: allow the QuadPlot class to plot multiple vehicles at once
# TODO: link the plotting functionality to the Quacopter class in some way, better for multi-agent animations
# TODO: save animation to file, put on GitHub home


class QuadPlot:
    """
    Class descriptor of a quadcopter animation object. Allows for animation of historical quadcopter state information.
    """
    def __init__(self, sim_state_data):
        """
        Class constructor
        :param sim_state_data: 13xN array of state information (will become Qx13xN for multi-vehicle case)
        """
        if str(np.shape(sim_state_data)[0]) == "13":
            pass
        elif str(np.shape(sim_state_data)[1]) == "13":
            sim_state_data = np.transpose(sim_state_data)
        else:
            raise Exception("The dimensions of the input data is incorrect, check and try again")

        sim_state_data = sim_state_data[:, ::8]                 # TODO: handle the downsampling in the main.py file, it's a simulation spec

        # initialize the 3-D animation/plotting structure
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        data = np.array([[sim_state_data[0, :], sim_state_data[1, :], sim_state_data[2, :]]])

        # declare line objects
        ax.plot([], [], [], '-', c='darkred')[0]                                   # quad arm
        ax.plot([], [], [], '-', c='midnightblue')[0]                                    # quad arm
        ax.plot([], [], [], '-', c='darkgrey', marker='o', markersize=1, markevery=3)[0]          # quad center
        ax.plot([], [], [], '.', c='red', markersize=1)[0]                      # waypoints
        ax.plot([], [], [], '.', c='gold', markersize=2)[0]                     # trailing line
        ax.view_init(elev=30.0, azim=285)
        ax.dist = 12.0

        self._q1_pos_obj = [ax.plot(dat[0:1, 0], dat[0:1, 1], dat[0:1, 2])[0] for dat in data]      # create quadcopter "lines" plotting objects
        self.set_limits(1.0, 1.0, 3.0)

        # save some information as properties for later use
        self._fig = fig
        self._pos_data = data
        self._full_state = sim_state_data

    def plot_quad_3d(self):
        """
        Main plotting function, makes the FuncAnimation call that runs the animation using the update_quad() callback function.
        :return:
        """
        t = time.time()
        ani = animation.FuncAnimation(self._fig, self.update_quad, np.shape(self._pos_data)[2],
                                      fargs=(self._pos_data, self._q1_pos_obj),
                                      interval=1, blit=False, repeat=False)
        plt.show()

    #
    # Animation helper functions
    #

    def set_limits(self, x, y, z):
        """
        Set initial figure boundaries in __init__(), labels the figure axes and title
        :param x: (> 0) maximum x bound
        :param y: (> 0) maximum y bound
        :param z: (> 0) maximum z bound
        :return: NONE, sets pre-existing figure properties
        """
        ax = plt.gca()
        ax.set_xlim3d([0.0, x])  # x
        ax.set_ylim3d([0.0, y])  # y
        ax.set_zlim3d([0.0, z])  # z

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Altitude')
        ax.set_title('Quadcopter Simulation')

    def plot_waypoints(self, wpts):
        # TODO: pre-plot the points that the quadcopter is trying to maneuver over
        pass

    def update_quad(self, itr, pos_data, pos_obj):
        """
        FuncAnimation() callback function that updates the lines in the the animation figure that was initialized in __init__()
        :param itr: integer counter of the simulation step
        :param pos_data: Qx3xN array of position data
        :param pos_obj: [] array NOT IN USE, will be line object corresponding to each quadcopter
        :return: NONE, sets pre-existing figure properties
        """
        quad_pos_world = self.quad_pos_world(self._full_state[:, itr])              # position in world frame coordinates

        ax = plt.gca()
        lines = ax.get_lines()          # 5 line objects per quadcopter [arm1, arm2, center of mass, waypoint, historical tail
        lines_data = [quad_pos_world[:, [0, 2]], quad_pos_world[:, [1, 3]], quad_pos_world[:, [4, 5]]]      # pairs data with correct line object

        # plot world coordinates of quadcopter arms
        for line_obj, line_data in zip(lines[0:3], lines_data):
            x, y, z = line_data
            line_obj.set_data(x, y)
            line_obj.set_3d_properties(z)

        # trailing line/history update
        lines[4].set_data(pos_data[0, 0:2, :itr])
        lines[4].set_3d_properties(pos_data[0, 2, :itr])

    def quad_pos_world(self, state, L=0.046, H=0.05):
        """
        Takes in quadcopter parameters and state information and returns the 3-D position in the world frame
        :param state: 13x1 quadcopter state at a specific simulation iteration
        :param L: length of quadcopter arm from center of mass in meters, assumes all arms are the same
        :param H: height of the quadcoper in meters
        :return: position of the vehicle in world frame
        """
        pos = state[0:3]
        q = state[6:10]
        rot = quadcopter.Quadcopter.quat2rot(q)        # express quaternion rotation as a rotation matrix, ZXY rotation type

        # homogeneous transform from body to world frame
        padding = np.array([0, 0, 0, 1])                    # allows rigid body rotation about a point wrt SO(3) definition
        wHb = np.concatenate((np.concatenate((rot, pos[:, None]), axis=1), padding[None, :]), axis=0)

        body_frame = np.transpose(np.array([
            [L, 0, 0, 1],
            [0, L, 0, 1],
            [-L, 0, 0, 1],
            [0, -L, 0, 1],
            [0, 0, 0, 1],
            [0, 0, H, 1]
        ]))
        world_frame = np.matmul(wHb, body_frame)
        quad_pos_world = world_frame[0:3, :]

        return quad_pos_world