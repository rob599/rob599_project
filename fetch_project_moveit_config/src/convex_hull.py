#!/usr/bin/env python3

# Import what we need
import rospy
import sys
import math
import numpy as np

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Header
from scipy.spatial import ConvexHull, Delaunay
from sympy import solve, Poly, Eq, Function, exp, Symbol
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class Convex_hull:
    def __init__(self):
        # Initialize Subscribers
        self.IM_array_sub = rospy.Subscriber('IM_pose_array',numpy_msg(Floats), self.best_fit_plane_callback, queue_size=1)

        # Initialize Publishers
        self.convex_hull_pub  = rospy.Publisher('convex_hull'           , PolygonStamped, queue_size=1)
        self.plane_poly_pub   = rospy.Publisher('best_fit_plane_poly'   , PolygonStamped, queue_size=1)
        self.plane_marker_pub = rospy.Publisher('best_fit_plane_marker' , Marker        , queue_size=1)


        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup PolygonStamped for convex hull of Interactive Markers (IM's)
        self.convex_hull_polygon = PolygonStamped()
        self.convex_hull_polygon.header = self.header

        # Setup PolygonStamped for best fit plane
        self.plane_polygon = PolygonStamped()
        self.plane_polygon.header = self.header

        # Create triangle_list marker to fill in best fit plane
        self.plane_marker = Marker()
        self.plane_marker.header = self.header
        self.plane_marker.type = Marker.TRIANGLE_LIST
        self.plane_marker.action = Marker.ADD
        self.plane_marker.id = 0
        self.plane_marker.scale.x = 1
        self.plane_marker.scale.y = 1
        self.plane_marker.scale.z = 1
        self.plane_marker.color.a = .8
        self.plane_marker.color.r = 1.0
        self.plane_marker.color.g = 0
        self.plane_marker.color.b = 0

        # Intialize X,Y, and Z lists for the IM positions
        self.X = []
        self.Y = []
        self.Z = []

        # Initialize a lists for the projected points of each IM.
        self.proj_x = []
        self.proj_y = []
        self.proj_z = []

        # Initialize coefficent values.Equation: ax+by+cz=d. Normal Vector: <a,b,c>
        self.a = None
        self.b = None
        self.c = None

        # Initialize the unit normal vector.
        self.u_n = None

        # Initialize the 3D to 2D sub-plane tranform Matrix(M) and its Inverse.
        self.M = None
        self.M_inv = None

        # Initialize the 2D sub-plane coodinates.
        self.coordinates_2D = None

        # Initialize the verticies of the convex hull of the IM's
        self.convex_hull_verticies = None


    def best_fit_plane_callback(self,arr_msg):
        # Conditional statement to process arr_msg
        if len(arr_msg.data) < 9:
            pass

        else:
            # Sepearately store the components (XYZ) of the IM's
            for i in range(int(len(arr_msg.data)/3)):
                self.X.append(arr_msg.data[i*3 + 0])
                self.Y.append(arr_msg.data[i*3 + 1])
                self.Z.append(arr_msg.data[i*3 + 2])

            XYZ = np.array([self.X,self.Y,self.Z])

            # Begin plane fit computation. Reference:https://stackoverflow.com/questions/12299540/
            tmp_A = []
            tmp_b = []
            for i in range(len(self.X)):
                tmp_A.append([self.X[i], self.Y[i], 1])
                tmp_b.append(self.Z[i])
            b = np.matrix(tmp_b).T
            A = np.matrix(tmp_A)
            fit = (A.T * A).I * A.T * b
            errors = b - A * fit

            # Equation: fit[0]*x + fit[1]*y + fit[2] = z
            # assign a,b, and c values by rearranging equation.
            # Rearranged Eq. : (-fit[0]/fit[2])*x + (-fit[1]/fit[2])*y + (1/fit[2])*z = 1
            self.a = float(-fit[0]/fit[2])
            self.b = float(-fit[1]/fit[2])
            self.c = float(1/fit[2])
            #print("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
            #print("Normal vector: < {0}, {1}, {2}> ".format(self.a,self.b,self.c))

            self.projection_on_plane()


    def projection_on_plane(self):
        # Create an origin point by using x and y pose of the first IM and solve for z.
        orig_x = self.X[0]
        orig_y = self.Y[0]
        orig_z = self.solve_for_z(orig_x, orig_y)

        # Create unit normal vector
        mag = math.sqrt(self.a**2 + self.b**2 + self.c**2)
        u_a = self.a / mag
        u_b = self.b / mag
        u_c = self.c / mag
        self.u_n = np.array([u_a, u_b, u_c])

        self.proj_x = []
        self.proj_y = []
        self.proj_z = []

        # forloop to compute the projections of each IM
        for i in range(len(self.X)):
            # Make a vector from the origin points to the IM's location.
            # v = Interactive_maker_location - orig (with respect to each dimension)
            vx = self.X[i] - orig_x
            vy = self.Y[i] - orig_y
            vz = self.Z[i] - orig_z
            v  = np.array([vx,vy,vz])

            # Take dot product of vector <vx,vy,vz> and unit normal vector <nx,ny,nz>
            # to get the scalar distance from the IM to the plane along the normal.
            dist =np.matmul(v,self.u_n)

            # Multiply the unit normal vector by the dist, and subtract from the IM pose.
            # Append to appropriate lists.
            self.proj_x.append(self.X[i] - dist*self.u_n[0])
            self.proj_y.append(self.Y[i] - dist*self.u_n[1])
            self.proj_z.append(self.Z[i] - dist*self.u_n[2])

        # Begin the transfer_matrix function
        self.transfer_matrix()


    def solve_for_z(self,x_value,y_value):
        z = Symbol('z')
        # Use solve function to solve the z value
        z_value = solve(self.a*x_value + self.b*y_value + self.c*z - 1, z)
        return float(z_value[0])


    def transfer_matrix(self):
        # Reference for transfer matrix approach: https://stackoverflow.com/questions/49769459
        # Create a vector in the bestfit plane using the first two projected points
        ABx = self.proj_x[0] - self.proj_x[1]
        ABy = self.proj_y[0] - self.proj_y[1]
        ABz = self.proj_z[0] - self.proj_z[1]

        # Normalize the componentes of the vector AB
        mag = math.sqrt(ABx**2 + ABy**2 + ABz**2)
        u_ABx = ABx / mag
        u_ABy = ABy / mag
        u_ABz = ABz / mag
        u_AB  = np.array([u_ABx, u_ABy, u_ABz])

        # Create second base vector (it is in unit and lies in the best fit plane)
        # Perform cross-product of u_AB and u_n
        V = np.cross(u_AB,self.u_n)

        # Now we can calculate the four basis points A, u=A+u_AB, v=A+V, n=A+u_n
        Ax = self.proj_x[1]
        Ay = self.proj_y[1]
        Az = self.proj_z[1]

        ux = Ax + u_ABx
        uy = Ay + u_ABy
        uz = Az + u_ABz

        vx = Ax + V[0]
        vy = Ay + V[1]
        vz = Az + V[2]

        nx = Ax + self.u_n[0]
        ny = Ay + self.u_n[1]
        nz = Az + self.u_n[2]

        # Set S matrix
        S = np.array([[Ax, ux, vx, nx],
                      [Ay, uy, vy, ny],
                      [Az, uz, vz, nz],
                      [1 , 1 , 1 , 1 ]])

        # Transformation maps into matrix below (D)
        D = np.array([[0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [1, 1, 1, 1]])

        # Inverse matrix of S
        S_inv = np.linalg.inv(S)

        # Transform Matrix!
        self.M = np.matmul(D,S_inv)

        # Inverse of Tranform Matrix
        self.M_inv = np.linalg.inv(self.M)

        # Begin dimension reduction function.
        self.dimension_reduction()


    def dimension_reduction(self):
        # Set coordinates_2D to equal an empty array.
        self.coordinates_2D = np.empty(shape=[len(self.X),2])

        # for loop to compute the dimension reduction of the IM's projections
        for i in range(len(self.X)):
            IM_projection = np.array([self.X[i], self.Y[i], self.Z[i], 1])
            reduction = np.matmul(self.M,IM_projection)

            # Store the 2D sub-plane coordinates
            self.coordinates_2D[i] = [reduction[0], reduction[1]]

        # Begin Convex_hull on self.coordinates_2D
        self.convex_hull()


    def convex_hull(self):
        # Run convex hull function on 2D sub-plane coordinates.
        hull = ConvexHull(self.coordinates_2D)

        plane_points = []
        IM_points    = []

        for e in hull.vertices:
            plane_points.append(Point32(self.proj_x[e],self.proj_y[e],self.proj_z[e]))
            IM_points.append(Point32(self.X[e],self.Y[e],self.Z[e]))

        # Assign IM_points to convex_hull_polygon and publish it.
        self.convex_hull_polygon.polygon.points = IM_points
        self.convex_hull_pub.publish(self.convex_hull_polygon)

        # Assign plane_points to plane_polygon and publish it.
        self.plane_polygon.polygon.points = plane_points
        self.plane_poly_pub.publish(self.plane_polygon)

        # Begin triangulation of the polygon
        self.triangulation_polygon()


    def triangulation_polygon(self):
        # Run Delaunay function on the self.coodinates to get all the vertices
        # of the trianglies in the the polygon.
        triangulation_points = []
        triangulation = Delaunay(self.coordinates_2D)

        for j in range(len(triangulation.vertices)):
            for e in triangulation.vertices[j]:
                triangulation_points.append(Point32(self.proj_x[e], self.proj_y[e], self.proj_z[e]))

        # Assign triangulation_points to plane_marker.points and publish
        self.plane_marker.points = triangulation_points
        self.plane_marker_pub.publish(self.plane_marker)

        # Begin clear_parameters function
        self.clear_parameters()

    def clear_parameters(self):
        # wipe the previous data from lists
        del self.X[:],self.Y[:],self.Z[:]
        del self.proj_x[:],self.proj_y[:],self.proj_z[:]



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('convex_hull')
    Convex_hull()
    rospy.spin()
