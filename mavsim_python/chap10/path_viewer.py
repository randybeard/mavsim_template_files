"""
mavsimPy: path drawing function
    - Beard & McLain, PUP, 2012
    - Update history:  
        1/8/2019 - RWB
"""
import sys
sys.path.append("..")
import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector

from tools.rotations import Euler2Rotation

class path_viewer():
    def __init__(self):
        self.scale = 4000
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Path Viewer')
        self.window.setGeometry(0, 0, 1000, 1000)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=self.scale, elevation=90, azimuth=0)
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.points, self.meshColors = self._get_mav_points()

    ###################################
    # public functions
    def update(self, path, state):
        """
        Update the drawing of the mav.

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
        rotated_points = self._rotate_points(self.points, R)
        translated_points = self._translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self._points_to_mesh(translated_points)

        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            if path.flag=='line':
                straight_line_object = self.straight_line_plot(path)
                self.window.addItem(straight_line_object)  # add straight line to plot
            else:  # path.flag=='orbit
                orbit_object = self.orbit_plot(path)
                self.window.addItem(orbit_object)
            # initialize drawing of triangular mesh.
            self.body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.meshColors, # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
            self.window.addItem(self.body)  # add body to plot
            self.plot_initialized = True

        # else update drawing on all other calls to update()
        else:
            # reset mesh using rotated and translated points
            self.body.setMeshData(vertexes=mesh, vertexColors=self.meshColors)

        # update the center of the camera view to the mav location
        #view_location = Vector(state.pe, state.pn, state.h)  # defined in ENU coordinates
        #self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()

    ###################################
    # private functions
    def _rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def _translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1,points.shape[1]]))
        return translated_points

    def _get_mav_points(self):
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

    def _points_to_mesh(self, points):
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

    def straight_line_plot(self, path):
        points = np.array([[path.line_origin.item(0),
                            path.line_origin.item(1),
                            path.line_origin.item(2)],
                           [path.line_origin.item(0) + self.scale * path.line_direction.item(0),
                            path.line_origin.item(1) + self.scale * path.line_direction.item(1),
                            path.line_origin.item(2) + self.scale * path.line_direction.item(2)]])
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        red = np.array([[1., 0., 0., 1]])
        path_color = np.concatenate((red, red), axis=0)
        object = gl.GLLinePlotItem(pos=points,
                                   color=path_color,
                                   width=2,
                                   antialias=True,
                                   mode='lines')
        return object

    def orbit_plot(self, path):
        N = 100
        red = np.array([[1., 0., 0., 1]])
        theta = 0
        points = np.array([[path.orbit_center.item(0) + path.orbit_radius,
                            path.orbit_center.item(1),
                            path.orbit_center.item(2)]])
        path_color = red
        for i in range(0, N):
            theta += 2 * np.pi / N
            new_point = np.array([[path.orbit_center.item(0) + path.orbit_radius * np.cos(theta),
                                   path.orbit_center.item(1) + path.orbit_radius * np.sin(theta),
                                   path.orbit_center.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
            path_color = np.concatenate((path_color, red), axis=0)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        object = gl.GLLinePlotItem(pos=points,
                                   color=path_color,
                                   width=2,
                                   antialias=True,
                                   mode='line_strip')
        return object

