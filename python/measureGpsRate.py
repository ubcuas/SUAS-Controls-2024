import serial
import time

# Replace '/dev/ttyUSB0' with your GPS device's serial port
serial_port = 'COM5'
# Replace 9600 with your GPS device's baud rate
baud_rate = 921600
# Duration over which to measure the data rate (in seconds)
measure_duration = 60

def measure_data_rate():
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            start_time = time.time()
            data_received = 0  # in bytes
            
            while time.time() - start_time < measure_duration:
                data = ser.readline()
                data_received += len(data)
                
                # Optional: Print the received data
                # print(data.decode('utf-8'), end='')
            
            # Calculate and print the data rate
            data_rate = data_received / measure_duration
            print(f"Data rate: {data_rate} bytes/second")
            print(f"Total Data received: {data_received}")
    
    except serial.SerialException as e:
        print(f"Error opening serial port {serial_port}: {e}")

if __name__ == "__main__":
    measure_data_rate()