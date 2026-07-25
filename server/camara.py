import socket
import struct
import cv2
import numpy as np

HOST = '0.0.0.0'
PORT = 1884

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
print(f"Servidor de video a escuta em {HOST}:{PORT}")

def recv_exact(conn, n):
    """Le exactamente n bytes do socket"""
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

try:
    while True:
        print("A aguardar ESP32...")
        conn, addr = server.accept()
        print(f"ESP32 ligado: {addr}")

        try:
            while True:
                # 1. Ler 4 bytes do tamanho
                header = recv_exact(conn, 4)
                if header is None:
                    break
                tamanho = struct.unpack('<I', header)[0]

                # 2. Ler os bytes do JPEG
                jpeg_bytes = recv_exact(conn, tamanho)
                if jpeg_bytes is None:
                    break

                # 3. Descodificar e mostrar
                img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    cv2.imshow('ESP32-CAM', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt

        except (ConnectionResetError, BrokenPipeError):
            print("ESP32 desligou")
        finally:
            conn.close()
            cv2.destroyAllWindows()

except KeyboardInterrupt:
    print("\nServidor a fechar...")
finally:
    server.close()