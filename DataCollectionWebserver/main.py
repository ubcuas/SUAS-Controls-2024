import socket

HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f'Listening on {HOST}:{PORT}')
    conn, addr = s.accept()
    with conn:
        print(f'Connected by {addr}')
        with open('data.txt', 'a') as f:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                print(f'Received data: {data.decode()}')
                f.write(data.decode())
