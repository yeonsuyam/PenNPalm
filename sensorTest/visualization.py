"""
 Simulation of a rotating 3D Cube
 Developed by Leonel Machava <leonelmachava@gmail.com>
http://codeNtronix.com
"""
import sys, math, pygame
from operator import itemgetter
import numpy as np
import time
import queue
from pyquaternion import Quaternion

class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def rotateQ(self, quaternion):
        # self_q = self.to_quaternion()
        # q0 = Quaternion(self_q)
        q1 = Quaternion(quaternion)
        # q = Quaternion.slerp(q0, q1, 1.0)
        # q = q0 * q1
        # p = q.vector.tolist()
        # print(p)
        # p = qh.rotate_point(self_q, quaternion).flatten()

        p = q1.rotate(np.array([self.x, self.y, self.z])).tolist()


        return Point3D(p[0], p[1], p[2])
        # return Point3D(p[1], p[2], p[3])

    def to_quaternion(self):
        vec = np.array([0, self.x, self.y, self.z], dtype = np.float64)
        # vec = vec[np.newaxis,:]
        return vec

    def rotateX(self, angle):
        """ Rotates the point around the X axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        y = self.y * cosa - self.z * sina
        z = self.y * sina + self.z * cosa
        return Point3D(self.x, y, z)

    def rotateY(self, angle):
        """ Rotates the point around the Y axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        z = self.z * cosa - self.x * sina
        x = self.z * sina + self.x * cosa
        return Point3D(x, self.y, z)

    def rotateZ(self, angle):
        """ Rotates the point around the Z axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x = self.x * cosa - self.y * sina
        y = self.x * sina + self.y * cosa
        return Point3D(x, y, self.z)

    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)

class Simulation:
    # def __init__(self, rollData, pitchData, yawData, win_width = 640, win_height = 480):
    def __init__(self, data, win_width = 640, win_height = 480):
        pygame.init()

        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("Simulation of a rotating 3D Cube (http://codeNtronix.com)")

        self.clock = pygame.time.Clock()

        self.vertices = [
            Point3D(-1,1,-1),
            Point3D(1,1,-1),
            Point3D(1,-1,-1),
            Point3D(-1,-1,-1),
            Point3D(-1,1,1),
            Point3D(1,1,1),
            Point3D(1,-1,1),
            Point3D(-1,-1,1)
        ]

        # Define the vertices that compose each of the 6 faces. These numbers are
        # indices to the vertices list defined above.
        self.faces  = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]

        # Define colors for each face
        self.colors = [(255,0,255),(255,0,0),(0,255,0),(0,0,255),(0,255,255),(255,255,0)]

        # self.sock_cli = Client()
        # self.sock_cli.connect()

        # self.rollData = rollData
        # self.pitchData = pitchData
        # self.yawData = yawData
        self.data = data

        # print("Connected to the server!")

    def _parse_data(self, data):
        acc = np.array(data[0:3])
        temp = np.array(data[3])
        gyro = np.array(data[4:7])
        mag = np.array(data[7:10])

        acc, gyro, mag = self.imu.calibrate(acc = acc, gyro = gyro, mag = mag)

        gyro = np.radians(gyro)
        acc = self._zero_pad_for_quater(acc)
        gyro = self._zero_pad_for_quater(gyro)
        mag = self._zero_pad_for_quater(mag)
        return acc, temp, gyro, mag

    def _zero_pad_for_quater(self, vec):
        return np.pad(vec, (1,0), 'constant', constant_values = 0)

    def run(self):
        """ Main Loop """
        prev_t = time.perf_counter()
        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.screen.fill((0,32,0))

            # self.q = np.array(self.request_orientation())
            self.q = np.array(self.request_orientation())

            # It will hold transformed vertices.
            t = []

            for v in self.vertices:
                # Rotate the point around X axis, then around Y axis, and finally around Z axis.
                # r = v.rotateX(self.roll).rotateY(self.pitch).rotateZ(self.yaw)
                r = v.rotateQ(self.q)
                # Transform the point from 3D to 2D
                p = r.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed vertices
                t.append(p)

            # Calculate the average Z values of each face.
            avg_z = []
            i = 0
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z) / 4.0
                avg_z.append([i,z])
                i = i + 1

            # Draw the faces using the Painter's algorithm:
            # Distant faces are drawn before the closer ones.
            for tmp in sorted(avg_z,key=itemgetter(1),reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
                pygame.draw.polygon(self.screen,self.colors[face_index],pointlist)

            pygame.display.flip()

    def request_orientation(self):
        while True:
            try:
                # dx, dy = xData.get(False), yData.get(False)
                # self.roll = self.rollData.get(False)
                # self.pitch = self.pitchData.get(False)
                # self.yaw = self.yawData.get(False)
                self.q  = list(self.data.get(False))
            except queue.Empty:
                pass
            else:
                # self.q = np.array(, dtype = np.float64)
                return self.q
        


if __name__ == "__main__":
    Simulation().run()
