import serial
import argparse
import time
import logging
import threading


class SerialControllerInterface:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.c1 = [b'0', b'0', b'1']
        self.c0 = [b'0', b'1', b'0']
    def update(self):

        s = 0
        while(True): 
            print("Aperte a tecla C para enviar um clear no alarme")
            print("Aperte a tecla S para mudar o valor do alarme")

            u = input()
            if u[0] == 'C' or u[0] == 'c':
                print("Comando de clear enviado pela serial")
                r = self.ser.write(b'P')
                time.sleep(1)
                r = self.ser.write(b'C')
                time.sleep(1)
                r = self.ser.write(b'A')
                time.sleep(1)
                r = self.ser.write(b'X')
            if u[0] == 'S' or u[0] == 's':
                if s < 2:
                    s = s + 1
                else:
                    s = 0
                print("Alarme alterado para: {}{}".format(self.c1[s],self.c0[s]))
                r = self.ser.write(b'P')
                time.sleep(1)
                r = self.ser.write(self.c1[s])
                time.sleep(1)
                r = self.ser.write(self.c0[s])
                time.sleep(1)
                r = self.ser.write(b'X')

if __name__ == '__main__':
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str, default='COM3')
    argparse.add_argument('-b', '--baudrate', type=int, default=115200)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} with {})".format(args.serial_port, args.baudrate))
    maquininha = SerialControllerInterface(args.serial_port, args.baudrate)

    while True:
        maquininha.update()
