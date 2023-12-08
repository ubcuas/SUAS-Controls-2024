import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import numpy as np

# Quaternion to Rotation Matrix Conversion
def quaternion_to_rotation_matrix(quaternion):
    """
    Convert a quaternion into a rotation matrix.

    Args:
    quaternion (array): A numpy array of shape (4,) representing the quaternion in the format [x, y, z, w]

    Returns:
    rotation_matrix (array): A 4x4 numpy array representing the rotation matrix.
    """
    w, x, y, z = quaternion

    xx, xy, xz = x * x, x * y, x * z
    yy, yz, zz = y * y, y * z, z * z
    wx, wy, wz = w * x, w * y, w * z

    rotation_matrix = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy), 0],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx), 0],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy), 0],
        [0, 0, 0, 1]  # Adding an extra row for the 4x4 matrix
    ])

    return rotation_matrix.transpose()

# Initialize Cube Vertices and Edges
def init_cube():
    vertices = [
        [1, -1, -1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, -1],
        [1, -1, 1],
        [1, 1, 1],
        [-1, -1, 1],
        [-1, 1, 1]
    ]

    edges = [
        (0,1),
        (1,2),
        (2,3),
        (3,0),
        (4,5),
        (5,6),
        (6,7),
        (7,4),
        (0,4),
        (1,5),
        (2,6),
        (3,7)
    ]

    return vertices, edges

# Draw the Cube
def draw_cube(vertices, edges):
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

# Main Function
def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glEnable(GL_DEPTH_TEST)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -100)
    scale_factor_x = 10  # Adjust these values as needed
    scale_factor_y = 10
    scale_factor_z = 10

    # Serial setup
    ser = serial.Serial('COM3', 115200)  # Adjust your COM port and baud rate

    vertices, edges = init_cube()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Read quaternion data from serial
        # Flush serial and read data
        ser.reset_input_buffer()
        
        # Read position data
        position_data = ['0']
        while position_data[0] != '4100':
            line = ser.readline()
            decoded_line = line.decode('utf-8').strip()
            position_data = decoded_line.split(',')

        # Read quaternion data
        quaternion_data = ['0']
        while quaternion_data[0] != '4097':
            line = ser.readline()
            decoded_line = line.decode('utf-8').strip()
            quaternion_data = decoded_line.split(',')

        try:
            # Apply position translation
            if position_data[0] == '4100':
                x, y, z = [float(val) for val in position_data[1:4]]
                # Apply the scale factors
                x *= scale_factor_x
                y *= scale_factor_y
                z *= scale_factor_z

                print("Scaled position data: ", x, y, z)

                glLoadIdentity()
                glTranslatef(x, y, z)
                glScalef(0.1, 0.1, 0.1)  # Adjust object scaling as needed

            # Apply rotation
            if quaternion_data[0] == '4097':
                q = [float(val) for val in quaternion_data[1:]]
                rotation_matrix = quaternion_to_rotation_matrix(q)
                glMultMatrixf(rotation_matrix)

        except Exception as e:
            print(f"Error: {e}")

        # Clear the screen and draw the cube
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        draw_cube(vertices, edges)
        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == "__main__":
    main()
