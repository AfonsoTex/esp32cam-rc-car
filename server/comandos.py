import socket
import time
import pygame
import datetime

HOST = '0.0.0.0'
PORT = 1883

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
server.settimeout(2.0)
print(f"Servidor a escuta em {HOST}:{PORT}")

pygame.init()
pygame.joystick.init()

joystick = None

def normalizar(valor):
    return (valor + 1) / 2

def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def processar_eventos():
    global joystick
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            if joystick is not None:
                joystick.quit()
            joystick = pygame.joystick.Joystick(event.device_index)
            joystick.init()
            print(f"[{ts()}] Comando detetado!")
        elif event.type == pygame.JOYDEVICEREMOVED:
            if joystick is not None:
                joystick.quit()
                joystick = None
                print(f"[{ts()}] Comando desligou!")

try:
    while True:
        processar_eventos()

        try:
            print(f"[{ts()}] Aguardando ESP32...")
            conn, addr = server.accept()
            print(f"[{ts()}] ESP32 ligado: {addr}")
        except socket.timeout:
            continue

        comando_anterior = None
        ultimo_envio = time.time()

        try:
            while True:
                processar_eventos()

                if joystick is not None:
                    pygame.event.pump()
                    direcao = joystick.get_axis(0)
                    acelerar = joystick.get_axis(5)
                    travar   = joystick.get_axis(4)

                    acelerar_norm = normalizar(acelerar)
                    travar_norm   = normalizar(travar)

                    if abs(direcao) < 0.1:
                        direcao = 0.0

                    move = acelerar_norm - travar_norm
                else:
                    move    = 0.0
                    direcao = 0.0

                comando = f"MOV:{move:.2f},DIR:{direcao:.2f}\n"

                try:
                    if comando != comando_anterior:
                        print(f"[DEBUG] A enviar: {comando.strip()}")
                        conn.send(comando.encode())
                        comando_anterior = comando
                        ultimo_envio = time.time()


                    elif time.time() - ultimo_envio > 0.2:
                        conn.send(b'HB\n')
                        ultimo_envio = time.time()

                        
                except OSError as e:
                    print(f"[{ts()}] Ligacao perdida: {e}")
                    break

                time.sleep(0.05)

        finally:
            conn.close()

except KeyboardInterrupt:
    print("\nServidor a fechar...")
finally:
    server.close()
    pygame.quit()
    print("Recursos libertados")