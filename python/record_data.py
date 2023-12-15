'''
Script to record data from the sensors. The data comes in via the serial port
and is written to a file. The data is written in a csv format.
'''

import serial
import time

# Open the serial port
ser = serial.Serial('COM3', 921600)  # Adjust your COM port and baud rate

# Open the file
filename = r'C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\UAS-PPES\RecordedData\UP_HOLD_DOWN_Bias_Added.csv'
file = open(filename, 'w')

# Write the header
header = 'P_Pos, P_Vel, Altitude, Pressure, Acc_X, Acc_Y, Acc_Z, Latitude, Longitude, Altitude_GPS, LOCK, Satellites, q_w, q_x, q_y, q_z\n'
file.write(header)

# Read and record the data until keyboard interrupt
try:
    while True:
        # Read data from serial port
        line = ser.readline()
        line = line.decode("utf-8")  # Decode byte string into Unicode
        line = line.rstrip()  # Strip \n and \r from string
        print(line)
        # Write data to file
        file.write(line + '\n')
        # Wait a bit before reading again
        # time.sleep(0.1)
except KeyboardInterrupt:
    print('Keyboard Interrupt')
    # Close the serial port
    ser.close()
    # Close the file
    file.close() 