import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import numpy as np

# Function to draw a vector
def draw_vector(vector):
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)  # Start at the origin
    glVertex3f(vector[0], vector[1], vector[2])  # End at the specified vector point
    glEnd()

# Main Function
def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glEnable(GL_DEPTH_TEST)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -2.5)

    # Serial setup
    ser = serial.Serial('COM3', 115200)  # Adjust your COM port and baud rate

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Read vector data from serial
        ser.reset_input_buffer()
        data = ['0']
        while data[0] != '4099':
            line = ser.readline()
            decoded_line = line.decode('utf-8').strip()
            data = decoded_line.split(',')
        
        if data[0] == '4099':
            try:
                vector = [float(val) for val in data[1:4]]
                print(vector)
            except ValueError:
                print("Invalid data received")
                continue

            # Clear the screen and draw the vector
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            draw_vector(vector)
            pygame.display.flip()
            pygame.time.wait(10)

if __name__ == "__main__":
    main()
