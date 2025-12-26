#! /usr/bin/env python3

import rospy
import sensor_msgs.msg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
from scipy.spatial.transform import Rotation as R

import sys

class Analysis():

    def __init__(self, filename = None):
        self.title = None
        self.column_labels = None
        self.data = None
        self.column_label_dict = None
        self.filename = filename
        if filename is not None:
            self.read(filename)
        

    def read(self, filename):
        self.filename = filename
        self.column_labels = np.genfromtxt(filename, delimiter=",", dtype='unicode', max_rows=1, autostrip=True)
        print(self.column_labels)
        self.column_label_dict = {}
        for i in range(len(self.column_labels)):
            self.column_label_dict[self.column_labels[i]] = i
        self.data = np.array(np.genfromtxt(filename, delimiter=",", dtype='unicode', skip_header=1, autostrip=True, unpack=True))

    def plot(self, save=False):
        if self.data is None:
            print("No data to plot! Run read(filename) first.")
            return
        
        # Plot arbitrary data for sanity checking
        column_numbers = range(12,16)
        plt.figure("Columns " + str(list(column_numbers)))
        t = self.data[0].astype(np.float64)
        for i in column_numbers:
            plt.plot(t, self.data[i].astype(np.float64))
        plt.legend([self.column_labels[i] for i in column_numbers], bbox_to_anchor=(1.04, 0.5), loc="center left")
        plt.tight_layout()
        
        # Plot distance to nearest collision
        plt.figure("Distance to collision")
        d = self.data[self.column_label_dict["nearest_collision_depth"]].astype(np.float64)
        plt.plot(t, d)

        # Plot joint states
        plt.figure("Joint states")
        joint_state_labels = [column_label for column_label in self.column_labels if "_joint" in column_label]
        for label in joint_state_labels:
            plt.plot(t, self.data[self.column_label_dict[label]].astype(np.float64))
        plt.legend(joint_state_labels, bbox_to_anchor=(1.04, 0.5), loc="center left")
        plt.tight_layout()

        # Prepare functions for animated plot
        def update_lines(step, data, plots):
            for plot, datum in zip(plots, data):
                plot.set_data(datum[:step,:2].T)
                plot.set_3d_properties(datum[:step,2])
            return plots
        
        def update_position_rotation(step, position_data_by_link, position_plots_by_link, rotation_matrices_by_link, rotation_plots_by_link):
            # For each step, get all previous position data and get the current rotation vector
            for position_plot, position_data, rotation_plots, rotation_matrices in zip(position_plots_by_link, position_data_by_link, rotation_plots_by_link, rotation_matrices_by_link):
                position_plot.set_data(position_data[:step,:2].T)
                position_plot.set_3d_properties(position_data[:step,2])
                position_plot.set_color("black")
                
                # Get axis vectors of the effector frame. Note they must be additionally rotated to match the control scheme.
                axis_vector_scale = 0.2
                x_vector = np.array(rotation_matrices[step-1,:,1].T) * axis_vector_scale
                y_vector = np.array(rotation_matrices[step-1,:,2].T) * axis_vector_scale
                z_vector = np.array(-rotation_matrices[step-1,:,0].T) * axis_vector_scale

                x_vector_data = np.array([position_data[step-1], np.add(position_data[step-1], x_vector)])
                y_vector_data = np.array([position_data[step-1], np.add(position_data[step-1], y_vector)])
                z_vector_data = np.array([position_data[step-1], np.add(position_data[step-1], z_vector)])

                rotation_plots[0].set_data(x_vector_data[:,:2].T)
                rotation_plots[0].set_3d_properties(x_vector_data[:,2])
                rotation_plots[0].set_color("red")

                rotation_plots[1].set_data(y_vector_data[:,:2].T)
                rotation_plots[1].set_3d_properties(y_vector_data[:,2])
                rotation_plots[1].set_color("green")

                rotation_plots[2].set_data(z_vector_data[:,:2].T)
                rotation_plots[2].set_3d_properties(z_vector_data[:,2])
                rotation_plots[0].set_color("blue")

            plots = [position_plots_by_link] + rotation_plots_by_link
            return position_plots_by_link

        # Set common info for plotting over time
        step_size = 5
        num_steps = len(self.data[0]) // step_size
        links = ["tcp_link"]

        # Set info for plotting position over time
        position_suffixes = ["_position_x", "_position_y", "_position_z"]
        position_labels_by_link = [[link + suffix for suffix in position_suffixes] for link in links]
        position_data_by_link = np.array([np.transpose([self.data[self.column_label_dict[label]].astype(np.float64) for label in position_labels]) for position_labels in position_labels_by_link])

        # Set info for plotting quaternion over time
        quaternion_suffixes = ["_quaternion_x", "_quaternion_y", "_quaternion_z", "_quaternion_w"]
        quaternion_labels_by_link = [[link + suffix for suffix in quaternion_suffixes] for link in links]
        quaternion_data_by_link = np.array([np.transpose([self.data[self.column_label_dict[label]].astype(np.float64) for label in quaternion_labels]) for quaternion_labels in quaternion_labels_by_link])

        # Get slice of data
        position_data_by_link = position_data_by_link[:,::step_size,:]
        quaternion_data_by_link = quaternion_data_by_link[:,::step_size,:]

        # Calculate directions of xyz axis lines
        rotation_matrices_by_link = np.empty(shape=(len(links), num_steps, 3, 3), dtype=np.float64)
        for i in range(len(links)):
            rotation_matrices = np.empty(shape=(num_steps, 3, 3), dtype=np.float64)
            for j in range(num_steps):
                rotation = R.from_quat(quaternion_data_by_link[i,j])
                rotation_matrices[j] = np.array(rotation.as_matrix())
            rotation_matrices_by_link[i] = rotation_matrices

        # Attaching 3D axis to the figure
        fig = plt.figure("tcp_link position")
        ax = fig.add_subplot(projection="3d")
        
        # Create lines initially without data
        position_plots_by_link = [ax.plot([], [], [])[0] for _ in links]
        rotation_plots_by_link = [[ax.plot([], [], [])[0] for i in range(3)] for _ in links]

        # Setting the axes properties
        # ax.set(xlim3d=(-1, 1), xlabel='X')
        # ax.set(ylim3d=(-1, 1), ylabel='Y')
        # ax.set(zlim3d=(-1, 1), zlabel='Z')
        # Creating the Animation object
        print("Creating animation")
        ani = animation.FuncAnimation(
            fig, update_position_rotation, num_steps, 
                fargs=(position_data_by_link, 
                position_plots_by_link, 
                rotation_matrices_by_link, 
                rotation_plots_by_link), 
                interval=50, repeat_delay=0)
        # Saving the Animation object
        if save:
            print("Saving animation")
            self.save_animation(ani)

        print("Showing plots")
        plt.show()

    def save_animation(self, ani):
        savedir = "/root/catkin_ws/src/morpheus_analysis/analysis/"
        writer = animation.FFMpegWriter(fps=10)
        ani.save(savedir + "test.mp4", writer=writer)
        return

        

if __name__=="__main__":

    filename = "/root/catkin_ws/src/morpheus_data/data/UR5e_simkeyhole_0_2024-03-12_16h49m22s.csv"
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    analysis = Analysis(filename)
    analysis.plot(save=True)
