"""
mavsim_python: world viewer (for chapter 12)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/3/2019 - BGM
"""
import sys
sys.path.append("..")
import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector

from tools.rotations import Euler2Rotation
from chap11.dubins_parameters import dubins_parameters

class world_viewer():
    def __init__(self):
        self.scale = 4000
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Path Viewer')
        self.window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=self.scale, elevation=50, azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.mav_points, self.mav_meshColors = self.get_mav_points()
        # dubins path parameters
        self.dubins_path = dubins_parameters()
        self.mav_body = []

    ###################################
    # public functions
    def update(self, map, waypoints, path, state):

        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.drawMAV(state)
            self.drawWaypoints(waypoints, path.orbit_radius)
            self.drawPath(path)
            self.drawMap(map)
            self.plot_initialized = True

        # else update drawing on all other calls to update()
        else:
            self.drawMAV(state)
            if waypoints.flag_waypoints_changed==True:
                self.drawWaypoints(waypoints, path.orbit_radius)
            if path.flag_path_changed==True:
                self.drawPath(path)

        # update the center of the camera view to the mav location
        #view_location = Vector(state.pe, state.pn, state.h)  # defined in ENU coordinates
        #self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()

    def drawMAV(self, state):
        """
        Update the drawing of the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.pn  # north position
            state.pe  # east position
            state.h   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        mav_position = np.array([[state.pn], [state.pe], [-state.h]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R.T)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        if not self.plot_initialized:
            # initialize drawing of triangular mesh.
            self.mav_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                  vertexColors=self.mav_meshColors,  # defines mesh colors (Nx1)
                                  drawEdges=True,  # draw edges between mesh elements
                                  smooth=False,  # speeds up rendering
                                  computeNormals=False)  # speeds up rendering
            self.window.addItem(self.mav_body)  # add body to plot
        else:
            # draw MAV by resetting mesh using rotated and translated points
            self.mav_body.setMeshData(vertexes=mesh, vertexColors=self.mav_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1,points.shape[1]]))
        return translated_points

    def get_mav_points(self):
        """"
            Points that define the mav, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define MAV body parameters
        unit_length = 0.25
        fuse_h = unit_length
        fuse_w = unit_length
        fuse_l1 = unit_length * 2
        fuse_l2 = unit_length
        fuse_l3 = unit_length * 4
        wing_l = unit_length
        wing_w = unit_length * 6
        tail_h = unit_length
        tail_l = unit_length
        tail_w = unit_length * 2

        #points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([[fuse_l1, 0, 0],  # point 1 [0]
                           [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],  # point 2 [1]
                           [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],  # point 3 [2]
                           [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],  # point 4 [3]
                           [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],  # point 5 [4]
                           [-fuse_l3, 0, 0],  # point 6 [5]
                           [0, wing_w / 2.0, 0],  # point 7 [6]
                           [-wing_l, wing_w / 2.0, 0],  # point 8 [7]
                           [-wing_l, -wing_w / 2.0, 0],  # point 9 [8]
                           [0, -wing_w / 2.0, 0],  # point 10 [9]
                           [-fuse_l3 + tail_l, tail_w / 2.0, 0], # point 11 [10]
                           [-fuse_l3, tail_w / 2.0, 0],  # point 12 [11]
                           [-fuse_l3, -tail_w / 2.0, 0],  # point 13 [12]
                           [-fuse_l3 + tail_l, -tail_w / 2.0, 0],   # point 14 [13]
                           [-fuse_l3 + tail_l, 0, 0],  # point 15 [14]
                           [-fuse_l3, 0, -tail_h],  # point 16 [15]
                           ]).T

        # scale points for better rendering
        scale = 50
        points = scale * points

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((13, 3, 4), dtype=np.float32)
        meshColors[0] = yellow  # nose-top
        meshColors[1] = yellow  # nose-right
        meshColors[2] = yellow  # nose-bottom
        meshColors[3] = yellow  # nose-left
        meshColors[4] = blue  # fuselage-left
        meshColors[5] = blue  # fuselage-top
        meshColors[6] = blue  # fuselage-right
        meshColors[7] = red  # fuselage-bottom
        meshColors[8] = green  # wing
        meshColors[9] = green  # wing
        meshColors[10] = green  # horizontal tail
        meshColors[11] = green  # horizontal tail
        meshColors[12] = blue  # vertical tail
        return points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points=points.T
        mesh = np.array([[points[0], points[1], points[2]],  # nose-top
                         [points[0], points[1], points[4]],  # nose-right
                         [points[0], points[3], points[4]],  # nose-bottom
                         [points[0], points[3], points[2]],  # nose-left
                         [points[5], points[2], points[3]],  # fuselage-left
                         [points[5], points[1], points[2]],  # fuselage-top
                         [points[5], points[1], points[4]],  # fuselage-right
                         [points[5], points[3], points[4]],  # fuselage-bottom
                         [points[6], points[7], points[9]],  # wing
                         [points[7], points[8], points[9]],  # wing
                         [points[10], points[11], points[12]],  # horizontal tail
                         [points[10], points[12], points[13]],  # horizontal tail
                         [points[5], points[14], points[15]],  # vertical tail
                        ])
        return mesh

    def drawPath(self, path):
        red = np.array([[1., 0., 0., 1]])
        if path.type == 'line':
            points = self.straight_line_points(path)
        elif path.type == 'orbit':
            points = self.orbit_points(path)
        if not self.plot_initialized:
            path_color = np.tile(red, (points.shape[0], 1))
            self.path = gl.GLLinePlotItem(pos=points,
                                          color=path_color,
                                          width=2,
                                          antialias=True,
                                          mode='line_strip')
                                          #mode='line_strip')
            self.window.addItem(self.path)
        else:
            self.path.setData(pos=points)

    def straight_line_points(self, path):
        points = np.array([[path.line_origin.item(0),
                            path.line_origin.item(1),
                            path.line_origin.item(2)],
                           [path.line_origin.item(0) + self.scale * path.line_direction.item(0),
                            path.line_origin.item(1) + self.scale * path.line_direction.item(1),
                            path.line_origin.item(2) + self.scale * path.line_direction.item(2)]])
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points

    def orbit_points(self, path):
        N = 10
        theta = 0
        theta_list = [theta]
        while theta < 2*np.pi:
            theta += 0.1
            theta_list.append(theta)
        points = np.array([[path.orbit_center.item(0) + path.orbit_radius,
                            path.orbit_center.item(1),
                            path.orbit_center.item(2)]])
        for angle in theta_list:
            new_point = np.array([[path.orbit_center.item(0) + path.orbit_radius * np.cos(angle),
                                   path.orbit_center.item(1) + path.orbit_radius * np.sin(angle),
                                   path.orbit_center.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points

    def drawWaypoints(self, waypoints, radius):
        blue = np.array([[0., 0., 1., 1.]])
        blue = np.array([[30, 144, 255, 255]])/255.
        if waypoints.type=='straight_line' or waypoints.type=='fillet':
            points = self.straight_waypoint_points(waypoints)
        elif waypoints.type=='dubins':
            points = self.dubins_points(waypoints, radius, 0.1)
        if not self.plot_initialized:
            waypoint_color = np.tile(blue, (points.shape[0], 1))
            self.waypoints = gl.GLLinePlotItem(pos=points,
                                               color=waypoint_color,
                                               width=2,
                                               antialias=True,
                                               mode='line_strip')
            self.window.addItem(self.waypoints)
        else:
            self.waypoints.setData(pos=points)

    def straight_waypoint_points(self, waypoints):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ waypoints.ned
        return points.T

    def dubins_points(self, waypoints, radius, Del):
        initialize_points = True
        for j in range(0, waypoints.num_waypoints-1):
            self.dubins_path.update(
                waypoints.ned[:, j],
                waypoints.course.item(j),
                waypoints.ned[:, j+1],
                waypoints.course.item(j+1),
                radius)

            # points along start circle
            th1 = np.arctan2(self.dubins_path.p_s.item(1) - self.dubins_path.center_s.item(1),
                            self.dubins_path.p_s.item(0) - self.dubins_path.center_s.item(0))
            th1 = mod(th1)
            th2 = np.arctan2(self.dubins_path.r1.item(1) - self.dubins_path.center_s.item(1),
                             self.dubins_path.r1.item(0) - self.dubins_path.center_s.item(0))
            th2 = mod(th2)
            th = th1
            theta_list = [th]
            if self.dubins_path.dir_s > 0:
                if th1 >= th2:
                    while th < th2 + 2*np.pi:
                        th += Del
                        theta_list.append(th)
                else:
                    while th < th2:
                        th += Del
                        theta_list.append(th)
            else:
                if th1 <= th2:
                    while th > th2 - 2*np.pi:
                        th -= Del
                        theta_list.append(th)
                else:
                    while th > th2:
                        th -= Del
                        theta_list.append(th)

            if initialize_points:
                points = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(theta_list[0]),
                                    self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(theta_list[0]),
                                    self.dubins_path.center_s.item(2)]])
                initialize_points = False
            for angle in theta_list:
                new_point = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(angle),
                                       self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(angle),
                                       self.dubins_path.center_s.item(2)]])
                points = np.concatenate((points, new_point), axis=0)

            # points along straight line
            sig = 0
            while sig <= 1:
                new_point = np.array([[(1 - sig) * self.dubins_path.r1.item(0) + sig * self.dubins_path.r2.item(0),
                                       (1 - sig) * self.dubins_path.r1.item(1) + sig * self.dubins_path.r2.item(1),
                                       (1 - sig) * self.dubins_path.r1.item(2) + sig * self.dubins_path.r2.item(2)]])
                points = np.concatenate((points, new_point), axis=0)
                sig += Del

            # points along end circle
            th2 = np.arctan2(self.dubins_path.p_e.item(1) - self.dubins_path.center_e.item(1),
                             self.dubins_path.p_e.item(0) - self.dubins_path.center_e.item(0))
            th2 = mod(th2)
            th1 = np.arctan2(self.dubins_path.r2.item(1) - self.dubins_path.center_e.item(1),
                             self.dubins_path.r2.item(0) - self.dubins_path.center_e.item(0))
            th1 = mod(th1)
            th = th1
            theta_list = [th]
            if self.dubins_path.dir_e > 0:
                if th1 >= th2:
                    while th < th2 + 2 * np.pi:
                        th += Del
                        theta_list.append(th)
                else:
                    while th < th2:
                        th += Del
                        theta_list.append(th)
            else:
                if th1 <= th2:
                    while th > th2 - 2 * np.pi:
                        th -= Del
                        theta_list.append(th)
                else:
                    while th > th2:
                        th -= Del
                        theta_list.append(th)
            for angle in theta_list:
                new_point = np.array([[self.dubins_path.center_e.item(0) + self.dubins_path.radius * np.cos(angle),
                                       self.dubins_path.center_e.item(1) + self.dubins_path.radius * np.sin(angle),
                                       self.dubins_path.center_e.item(2)]])
                points = np.concatenate((points, new_point), axis=0)

        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points

    def drawMap(self, map):
        # draw map of the world: buildings
        fullMesh = np.array([], dtype=np.float32).reshape(0,3,3)
        fullMeshColors = np.array([], dtype=np.float32).reshape(0,3,4)
        for i in range(0, map.num_city_blocks):
            for j in range (0, map.num_city_blocks):
                mesh, meshColors = self.buildingVertFace(map.building_north[i],
                                                      map.building_east[j],
                                                      map.building_width,
                                                      map.building_height[j, i])
                fullMesh = np.concatenate((fullMesh, mesh), axis=0)
                fullMeshColors = np.concatenate((fullMeshColors, meshColors), axis=0)
        self.map = gl.GLMeshItem(vertexes= fullMesh,  # defines the triangular mesh (Nx3x3)
                      vertexColors= fullMeshColors,  # defines mesh colors (Nx1)
                      drawEdges=True,  # draw edges between mesh elements
                      smooth=False,  # speeds up rendering
                      computeNormals=False)  # speeds up rendering
        self.window.addItem(self.map)

    def buildingVertFace(self, n, e, width, height):
        # define patches for a building located at (x, y)
        # vertices of the building
        points = np.array([[e + width / 2, n + width / 2, 0], #NE 0
                         [e + width / 2, n - width / 2, 0],   #SE 1
                         [e - width / 2, n - width / 2, 0],   #SW 2
                         [e - width / 2, n + width / 2, 0],   #NW 3
                         [e + width / 2, n + width / 2, height], #NE Higher 4
                         [e + width / 2, n - width / 2, height], #SE Higher 5
                         [e - width / 2, n - width / 2, height], #SW Higher 6
                         [e - width / 2, n + width / 2, height]]) #NW Higher 7
        mesh = np.array([[points[0], points[3], points[4]],  #North Wall
                         [points[7], points[3], points[4]],  #North Wall
                         [points[0], points[1], points[5]],  # East Wall
                         [points[0], points[4], points[5]],  # East Wall
                         [points[1], points[2], points[6]],  # South Wall
                         [points[1], points[5], points[6]],  # South Wall
                         [points[3], points[2], points[6]],  # West Wall
                         [points[3], points[7], points[6]],  # West Wall
                         [points[4], points[7], points[5]],  # Top
                         [points[7], points[5], points[6]]])  # Top

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((10, 3, 4), dtype=np.float32)
        meshColors[0] = green
        meshColors[1] = green
        meshColors[2] = green
        meshColors[3] = green
        meshColors[4] = green
        meshColors[5] = green
        meshColors[6] = green
        meshColors[7] = green
        meshColors[8] = yellow
        meshColors[9] = yellow
        return mesh, meshColors


def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x
