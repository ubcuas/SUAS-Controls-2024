import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import serial

def quaternion_to_euler(q0, q1, q2, q3):
    # Roll (x-axis rotation)
    t0 = +2.0 * (q0 * q1 + q2 * q3)
    t1 = +1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(t0, t1) * 180.0 / np.pi

    # Pitch (y-axis rotation)
    t2 = +2.0 * (q0 * q2 - q3 * q1)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2) * 180.0 / np.pi

    # Yaw (z-axis rotation)
    t3 = +2.0 * (q0 * q3 + q1 * q2)
    t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(t3, t4) * 180.0 / np.pi

    return roll, pitch, yaw

def draw_text(position, text):
    font = pygame.font.SysFont("Arial", 18, True)
    text_surface = font.render(text, True, (255, 255, 255, 255))
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)

def draw_cube(color, scale=1.0):
    vertices = [
        [scale, scale, scale],
        [-scale, scale, scale],
        [-scale, -scale, scale],
        [scale, -scale, scale],
        [scale, scale, -scale],
        [-scale, scale, -scale],
        [-scale, -scale, -scale],
        [scale, -scale, -scale]
    ]

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Front face
        (4, 5), (5, 6), (6, 7), (7, 4),  # Back face
        (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges
    ]

    glColor3fv(color)
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def read_serial_data(port, baudrate):
    ser = serial.Serial(port, baudrate=baudrate, timeout=1)
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                ser.close()
                quit()
        ser.flushInput()
        line = ser.readline()
        line = line.decode("utf-8")  # Decode byte string into Unicode
        line = line.rstrip()  # Strip \n and \r from string
        print(line) 
        if line:
            try:
                data = line.split(',')
                # print(len(data))
                if data[0] == 'head' and len(data) == 9:
                    q0 = float(data[1])
                    q1 = float(data[2])
                    q2 = float(data[3])
                    q3 = float(data[4])
                    accuracy = int(data[5])
                    roll_in = float(data[6])
                    pitch_in = float(data[7])
                    yaw_in = float(data[8])

                    roll_comp, pitch_comp, yaw_comp = quaternion_to_euler(q0, q1, q2, q3)

                    roll_diff = roll_in - roll_comp
                    pitch_diff = pitch_in - pitch_comp
                    yaw_diff = yaw_in - yaw_comp

                    print(f"Received: roll={roll_in}, pitch={pitch_in}, yaw={yaw_in}")
                    print(f"Computed: roll={roll_comp}, pitch={pitch_comp}, yaw={yaw_comp}")
                    print(f"Differences: roll_diff={roll_diff}, pitch_diff={pitch_diff}, yaw_diff={yaw_diff}")
                    print()

                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
                    # roll_in = roll_in * np.pi/180.0
                    # roll_comp = roll_comp * np.pi/180.0
                    # pitch_in = pitch_in * np.pi/180.0
                    # pitch_comp = pitch_comp * np.pi/180.0
                    # yaw_in = yaw_in * np.pi/180.0
                    # yaw_comp = yaw_comp * np.pi/180.0
                    # Draw received orientation
                    # Draw received orientation
                    # Draw received orientation
                    glPushMatrix()
                    glTranslatef(-1, 0, 0)
                    glRotatef(roll_in, 1, 0, 0)
                    glRotatef(pitch_in, 0, 1, 0)
                    glRotatef(yaw_in, 0, 0, 1)
                    draw_cube((1, 0, 0), scale=0.5)  # Red cube for received orientation
                    glPopMatrix()

                    # Draw computed orientation
                    glPushMatrix()
                    glTranslatef(1, 0, 0)
                    glRotatef(roll_comp, 1, 0, 0)
                    glRotatef(pitch_comp, 0, 1, 0)
                    glRotatef(yaw_comp, 0, 0, 1)
                    draw_cube((0, 0, 1), scale=0.5)  # Blue cube for computed orientation
                    glPopMatrix()

                    # Draw text labels
                    glColor3fv((1, 1, 1))  # White color for text
                    draw_text((-3, 2, 0), "Received Orientation")
                    draw_text((3, 2, 0), "Computed Orientation")

                    pygame.display.flip()
            except ValueError as e:
                print(f"Error parsing data: {e}")

if __name__ == "__main__":
    read_serial_data('COM6', 921600)  # Replace 'COM6' with your actual serial port
