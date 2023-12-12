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
    w, y, x, z = quaternion

    z = -z  # Invert the z-axis

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

# Subtract Quaternions
def subtract_quaternion(q1, q2):
    """
    Subtract quaternion q2 from q1.

    Args:
    q1, q2 (array): Quaternions in the format [x, y, z, w]

    Returns:
    result (array): The resulting quaternion after subtraction.
    """
    result = np.zeros(4)
    result[0] = q1[0] - q2[0]
    result[1] = q1[1] - q2[1]
    result[2] = q1[2] - q2[2]
    result[3] = q1[3] - q2[3]
    return result

def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles.

    Args:
    R (array): A 3x3 or 4x4 numpy array representing the rotation matrix.

    Returns:
    (roll, pitch, yaw): A tuple of Euler angles (in radians).
    """
    sy = np.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)

# Main Function
def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glEnable(GL_DEPTH_TEST)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -10)

    # Serial setup
    ser = serial.Serial('COM3', 921600)  # Adjust your COM port and baud rate

    vertices, edges = init_cube()

    # Calibrate the sensor
    print("Calibrating... Please wait.")
    initial_quaternions = []
    for _ in range(100):
        # Read quaternion data from serial
        ser.reset_input_buffer()
        data = ['0', '0']
        while data[0] != '4097' and data[1] != 'nan':
            line = ser.readline()
            decoded_line = line.decode('utf-8').strip()
            data = decoded_line.split(',')
        try:
            if data[0] == '4097':
                data = data[1:]
                q = [float(val) for val in data]
                initial_quaternions.append(q)
        except Exception as e:
            print(f"Error: {e}")

    # Averaging the quaternions
    initial_orientation = np.mean(initial_quaternions, axis=0)

    # while True:
    #     print("Initial orientation: ", initial_orientation)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Read quaternion data from serial
        #flush serial
        ser.reset_input_buffer()
        #read until header
        data = ['0']
        while data[0] != '4097':
            line = ser.readline()
            decoded_line = line.decode('utf-8').strip()
            data = decoded_line.split(',')
        try:    
            # decoded_line = line.decode('utf-8').strip()
            # data = decoded_line.split(',')
            # # Check for header
            if data[0] == '4097':
                data = data[1:]
                # data = decoded_line[1:].split(',')
                print("data after removing top: " , data)
                q = [float(val) for val in data]
                # q = subtract_quaternion(q, initial_orientation)
                rotation_matrix = quaternion_to_rotation_matrix(q)
                rotation_matrix_intial = quaternion_to_rotation_matrix(initial_orientation)
                #get euler angles
                euler_angles = rotation_matrix_to_euler_angles(rotation_matrix)
                euler_angles_initial = rotation_matrix_to_euler_angles(rotation_matrix_intial)
                #subtract initial euler angles
                euler_angles = np.subtract(euler_angles, euler_angles_initial)
                print("Euler angles: ", euler_angles)
                glLoadIdentity()
                glScalef(0.5, 0.5, 0.5)  # Scale down the object; adjust the values as needed
                glTranslatef(0.0, 0.0, 0)
                # glMultMatrixf(rotation_matrix)
                # Apply rotations
                yaw, pitch, roll = euler_angles
                glRotatef(yaw, 0, 1, 0)   # Rotate around Y-axis
                glRotatef(pitch, 1, 0, 0) # Rotate around X-axis
                glRotatef(roll, 0, 0, 1)  # Rotate around Z-axis

        except Exception as e:
            print(f"Error: {e}")

        # Clear the screen and draw the cube
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        draw_cube(vertices, edges)
        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == "__main__":
    main()
