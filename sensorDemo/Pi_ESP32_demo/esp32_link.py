import serial
import threading
from queue import Queue

# For USB connection
# PORT = '/dev/ttyUSB0'
# For direct pin connection
# PORT = '/dev/ttyS0'
PORT = '/dev/ttyAMA0'
BAUDRATE = 57600
esp_serial = serial.Serial(PORT, BAUDRATE)

dropped_bottles = Queue()


def send_recv_drop_data(lat, lon, heading, bottleID):
    '''!Sends drop data to ESP32 and receives adjusted drop data
    
    @param lat in degrees
    @param lon in degrees
    @param heading degrees from north
    @param bottleID integer from 1-5
    
    @returns adjusted lat, lon, heading, bottleID
    '''
    esp_serial.Timeout = 2 # seconds
    
    data = f"{lat},{lon},{heading},{bottleID}"
    esp_serial.write(data.encode('utf-8'))

    try:
        adj_drop_data = esp_serial.readline().decode('utf-8').strip().split(",")
        print(adj_drop_data)
        return adj_drop_data
    except Exception as e: # Timeout probably
        print(e)
        return None

def read_serial():
    '''!Blocks until reads something from serial
    Modified dropped_bottles (by appending string indicating bottle that has dropped)
    '''
    esp_serial.Timeout = 5*60 # Wait 5 minutes for bottle to be dropped
    confirmation = esp_serial.readline().strip().decode('utf-8')
    dropped_bottles.put(confirmation)
    print([item for item in dropped_bottles.queue])

def monitor_drop_status():
    '''!Non-blocking function to wait for dropped confirmation message.'''
    thread = threading.Thread(target=read_serial)
    thread.daemon = True  # To avoid blocking program exit
    thread.start()


# Mock main program

adj_drop_data = send_recv_drop_data(40.1, -112.4, 23.4, 2)
monitor_drop_status()

while True:
    pass # Do other things (ODLC probably)