# This is the beginning of script to read and plot the current orientation of the IMU

import socket
import numpy as np
import pygame
import math
import time
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

HOST = "ip_address"  # Ghost detector IP address
PORT = 9005  # SocketHost listening port

video_flags = OPENGL | DOUBLEBUF

def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def draw(quat):
    w = quat[0]
    nx = quat[1]
    ny = quat[2]
    nz = quat[3]

    print(w, nx, ny, nz)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "PyTeapot", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -7.867  # Declination in Boulder is 7deg 52'
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("PyTeapot IMU orientation visualization")
    resizewin(640, 480)
    init()

    s.connect((HOST, PORT))

    print(f"Connected to GHRS at {HOST}.")

    # Read new data
    s.sendall(b'\x01\x02\xDE\xAD')
    # data = s.recv(32)
    data = s.recv(64)
    dt = np.dtype(float)
    dt = dt.newbyteorder('>')

    initial_quat = np.frombuffer(data, dtype=dt, count=4)

    frames = 0
    ticks = pygame.time.get_ticks()
    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        # Read new data
        s.sendall(b'\x01\x02\xDE\xAD')
        # data = s.recv(32)
        data = s.recv(64)
        print(len(data))
        dt = np.dtype(float)
        dt = dt.newbyteorder('>')
        print(data)
        if not data:
            break

        quat = np.frombuffer(data, dtype=dt, count=4)

        # transformed_quat = q_mult(q_mult(initial_quat, quat), q_conjugate(initial_quat))
        # transformed_quat = q_mult(q_conjugate(initial_quat), quat)

        # Draw a new frame
        draw(quat)
        pygame.display.flip()
        frames += 1

        time.sleep(1.0/30.0)

    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
