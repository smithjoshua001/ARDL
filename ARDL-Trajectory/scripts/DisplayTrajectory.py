#!/usr/bin/python
import DynamicsLib
import numpy as np
import ModifiedFourierTrajectory
import OptimizerBase
import MFTOptimizer
import argparse
from math import ceil


def main():
    parser = argparse.ArgumentParser(
        description='Optimize a modified fourier trajectory with respect to the condition number. Considering constraints.')
    parser.add_argument('urdf_file', metavar='file_name', type=str,
                        nargs=1, help='URDF file of the robot for structure information')
    parser.add_argument('--input-file', dest='ifile', type=str,
                        help='input file')
    args = parser.parse_args()
    chain = DynamicsLib.Chain_d(args.urdf_file[0])
    kin = DynamicsLib.ForwardKinematics_d(chain)

    trajModel = ModifiedFourierTrajectory.ModifiedFourierTrajectory_d(
        chain.getNumOfJoints())
    ifile = str(args.ifile)
    trajModel.loadFromJSON(ifile)
    # params = np.zeros(trajModel.getNumberOfParameters())

    control_rate = 100
    dof = chain.getNumOfJoints()
    import matplotlib.pyplot as plt
    import mpl_toolkits.mplot3d as mplot3d
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    trajectory_data = trajModel.simulate(control_rate)
    q_discrete = np.zeros((int(ceil(trajModel.getPeriodLength()
                                    * control_rate)), dof))
    qd_discrete = np.zeros((
        int(ceil(trajModel.getPeriodLength()
                 * control_rate)), dof))
    FK_discrete = np.zeros((chain.getNumOfLinks(),
                            int(ceil(trajModel.getPeriodLength()
                                     * control_rate)), 3))
    FK_rot_discrete = np.zeros((chain.getNumOfLinks(),
                                int(ceil(trajModel.getPeriodLength()
                                         * control_rate)), 3, 3))
    collision = False
    adjv = DynamicsLib.AdjointSE3Vector(chain.getNumOfLinks())
    # np.set_printoptions(threshold=np.nan, suppress=True)
    # print trajectory_data.pos.T
    # exit(1)
    for k in range(0, int(ceil(trajModel.getPeriodLength() * control_rate))):
        q = trajectory_data.pos[:, k].copy().T

        qd = trajectory_data.vel[:, k].copy().T

        qdd = trajectory_data.acc[:, k].copy().T

        trajModel.getPeriodLength()

        chain.update(q,qd)

        kin.getLinkAdjointsVector(adjv,True)

        q_discrete[k, :] = q.T
        qd_discrete[k, :] = qd.T
        for i in range(0, chain.getNumOfLinks()):
            FK_discrete[i, k, :] = adjv[i].getP().T
            FK_rot_discrete[i, k, :, :] = adjv[i].getR()
    i = chain.getNumOfLinks() - 1
    ax.plot(FK_discrete[i, :, 0],
            FK_discrete[i, :, 1], FK_discrete[i, :, 2], label=i, marker='o', linewidth=2.0, markevery=100)
    ax.plot(FK_discrete[i - 1, :, 0],
            FK_discrete[i - 1, :, 1], FK_discrete[i - 1, :, 2], label=i, marker='o', linewidth=2.0, markevery=100)

    # from stl import mesh
    # import pymesh
    import math
    from copy import copy, deepcopy

    polies = []
    lines = []
    meshes = []
    prev = np.array([0, 0, 0])
    links = chain.getLinksRef()
    for i in range(chain.getNumOfLinks()):
        filename = chain.getCollisionFile(i)
        print i
        if(filename != ""):
            # if(filename == "CYLINDER"):

            #     # need to do work on
            #     # ax.plot([FK_discrete[i-1, 0, 0], FK_discrete[i, 0, 0]],
            #     # [FK_discrete[i-1, 0, 1], FK_discrete[i, 0, 1]],
            #     # zs=[FK_discrete[i-1, 0, 2], FK_discrete[i, 0, 2]])
            #     print 'CYL!'

            #     your_mesh = pymesh.generate_cylinder(np.array([0, 0, 0]),
            #                                          np.array([0, 0, links[i].getLength()]), links[i].getRadius(), links[i].getRadius())
            #     # if(i + 1 != chain.getNumOfLinks()):
            #     #     your_mesh = pymesh.generate_cylinder(
            #     # FK_discrete[i, 0, :], FK_discrete[i + 1, 0, :], 0.05, 0.05)
            #     pymesh.save_mesh('.tmp.stl', your_mesh)
            #     your_mesh = mesh.Mesh.from_file('.tmp.stl')
            #     meshes.append(mesh.Mesh.from_file('.tmp.stl'))
            #     if(i != chain.getNumOfLinks()):
            #         for j in range(3):
            #             your_mesh.vectors[:, j] = your_mesh.vectors[:, j].dot(
            #                 FK_rot_discrete[i, 0, :, :])

            #         your_mesh.translate(FK_discrete[i, 0, :])
            #         print FK_discrete[i, 0, :]
            #         pol = mplot3d.art3d.Poly3DCollection(
            #             your_mesh.vectors)
            #         polies.append(pol)
            #         ax.add_collection3d(pol)

            # else:
            #     your_mesh = mesh.Mesh.from_file(chain.getCollisionFile(i))
            #     meshes.append(mesh.Mesh.from_file(chain.getCollisionFile(i)))
            #     if(i != chain.getNumOfLinks()):
            #         for j in range(3):
            #             your_mesh.vectors[:, j] = your_mesh.vectors[:, j].dot(
            #                 FK_rot_discrete[i, 0, :, :])

            #         your_mesh.translate(FK_discrete[i, 0, :])
            #         pol = mplot3d.art3d.Poly3DCollection(
            #             your_mesh.vectors)
            #         polies.append(pol)
            #         ax.add_collection3d(pol)
            print("getting collision Vectors")
            print chain.getLink(i).getName()
            verts=chain.getLink(i).getCollisionVectors()
            tris= chain.getLink(i).getCollisionTriangles()
            your_mesh=  [[verts[tris[ix][iy]] for iy in range(len(tris[0]))] for ix in range(len(tris))];
            print("appending_mesh")
            meshes.append(deepcopy(your_mesh))
            print("transform!")
            # print i
            # print your_mesh
            # if(i != chain.getNumOfLinks()):
            # print pymesh.generate_cylinder(np.array([0, 0, 0]), np.array([0, 0, 0.5]), 0.1, 0.1).vectors[:,0]
            # print FK_rot_discrete[max(0,i-1), 1570, :, :]
            print chain.getLink(i).getCollisionRotation()
            # print FK_discrete[max(0,i-1), 1570, :]
            print chain.getLink(i).getCollisionTranslation().T[0]
            for j in range(3):
                for x in range(len(your_mesh)):
                    # your_mesh[x][j] = FK_rot_discrete[max(0,i-1), 1189, :, :].dot(your_mesh[x][j]) + FK_discrete[max(0,i-1), 1189, :]
                    your_mesh[x][j] = chain.getLink(i).getCollisionRotation().dot(your_mesh[x][j]) + chain.getLink(i).getCollisionTranslation().T[0]
            pol = mplot3d.art3d.Poly3DCollection(
                your_mesh,linewidths=1,facecolors='w',alpha=0.5)
            polies.append(pol)
            lin = mplot3d.art3d.Line3DCollection(your_mesh,colors='k',linewidths=0.2)
            lines.append(lin)
            ax.add_collection3d(pol)
            ax.add_collection3d(lin)


    class animation(object):

        def __init__(self, ax, polies, meshes):
            self.ax = ax
            self.ax.set_xlim(-1, 1)
            self.ax.set_ylim(-1, 1)
            self.ax.set_zlim(0, 2)
            self.polies = polies
            self.meshes = meshes

        def update(self, frame):
            print frame
            # your_mesh2 = deepcopy(self.meshes[0])
            # for j in range(3):
            #     your_mesh2.vectors[:, j] = your_mesh2.vectors[:, j].dot(
            #         FK_rot_discrete[7, frame, :, :])

            # your_mesh2.translate(FK_discrete[7, frame, :])

            # self.polies[0].set_verts(your_mesh2.vectors)
            # self.ax.add_collection3d(self.polies[0])

            return self.polies[0],

    animationt = animation(ax, polies, meshes)
    from matplotlib.animation import FuncAnimation
    # ani = FuncAnimation(fig, animationt.update, interval=5,
    #                     frames=range(100), blit=False)

    view_2D = False

    def move_view(event):
                # I have no idea, it this line have some effect at all
        ax.autoscale(enable=False, axis='both')
        # Set nearly similar speed of motion in dependency on zoom
        if view_2D:
            koef = 4.  # Speed for 2D should be higher
        else:
            koef = 8.  # speed for 3D should be lower
            zkoef = (ax.get_zbound()[0] - ax.get_zbound()[1]) / koef

        xkoef = (ax.get_xbound()[0] - ax.get_xbound()[1]) / koef
        ykoef = (ax.get_ybound()[0] - ax.get_ybound()[1]) / koef

        # Map an motion to keyboard shortcuts
        if event.key == "ctrl+down":
            ax.set_ybound(ax.get_ybound()[
                0] + xkoef, ax.get_ybound()[1] + xkoef)
        if event.key == "ctrl+up":
            ax.set_ybound(ax.get_ybound()[
                0] - xkoef, ax.get_ybound()[1] - xkoef)
        if event.key == "ctrl+right":
            ax.set_xbound(ax.get_xbound()[
                0] + ykoef, ax.get_xbound()[1] + ykoef)
        if event.key == "ctrl+left":
            ax.set_xbound(ax.get_xbound()[
                0] - ykoef, ax.get_xbound()[1] - ykoef)
        if not view_2D:
            if event.key == "down":
                ax.set_zbound(ax.get_zbound()[
                    0] - zkoef, ax.get_zbound()[1] - zkoef)
            if event.key == "up":
                ax.set_zbound(ax.get_zbound()[
                    0] + zkoef, ax.get_zbound()[1] + zkoef)

        fig.canvas.draw()
        fig.canvas.flush_events()

        # print event.key

    class Ani(object):

        def __init__(self):
            self.frame = 0

        def update(self):
            for i in range(len(meshes)):
                your_mesh2 = deepcopy(meshes[i])
                q = trajectory_data.pos[:, self.frame].copy().T
                qd = trajectory_data.vel[:, self.frame].copy().T
                qdd = trajectory_data.acc[:, self.frame].copy().T
                chain.update(q,qd)
                kin.getLinkAdjointsVector(adjv,True)
                # print kin.collision()
                for j in range(3):
                    for x in range(len(your_mesh2)):
                        # your_mesh2[x][j] = FK_rot_discrete[max(0,i-1), self.frame, :, :].dot(your_mesh2[x][j]) + FK_discrete[max(0,i-1), self.frame, :]
                        your_mesh2[x][j] = chain.getLink(i).getCollisionRotation().dot(your_mesh2[x][j]) + chain.getLink(i).getCollisionTranslation().T[0]

                # your_mesh2.translate(FK_discrete[i, self.frame, :])
                polies[i].set_verts(your_mesh2)
                lines[i].set_segments(your_mesh2)

            fig.canvas.draw()
            # print time.clock()-start_time
            # fig.canvas.flush_events()
            self.frame += 16
            (w, h, g) = FK_discrete.shape
            self.frame %= h

    import functools
    fig.canvas.mpl_connect("key_press_event", move_view)
    timer = fig.canvas.new_timer(interval=16)
    animation = Ani()
    print animation.frame
    timer.add_callback(Ani.update.__get__(animation))
    timer.start()
    # ax.legend()
    print collision

    plt.show()


if __name__ == "__main__":
    main()
