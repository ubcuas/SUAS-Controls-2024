import socket
import csv
from datetime import datetime

HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f'Listening on {HOST}:{PORT}')
    conn, addr = s.accept()
    with conn:
        print(f'Connected by {addr}')
        filename = f"data.csv"
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["TimeStamp", "TimeSent", "Data"])
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                print(f'Received data: {data.decode()}')
                now = datetime.now()
                dt_string = now.strftime("%d/%m/%Y %H:%M:%S,%f")
                #split data by new line
                data = data.decode().split("\n")
                #write data to csv file
                for line in data:
                    line = line.split(",")
                    data = [dt_string] + line
                    writer.writerow(data)