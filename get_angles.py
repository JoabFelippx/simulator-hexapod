import serial
import numpy as np

# Substitua por sua porta, ex:
#   - Windows: 'COM3'
#   - Linux/Mac: '/dev/ttyUSB0' ou '/dev/ttyACM0'
ser = serial.Serial('/dev/ttyUSB0', 38400)

def reading():
    if ser.in_waiting > 0:
        line = ser.readline().decode().strip()

        # print(line)
        if line[0] == '[' and line[-1] == ']':
            line = line.strip('[]') 
            a = line.split(',')
            if len(a) != 18:
                return None
            # print('a:',a)
            a = np.array([float(b) for b in a])
            a *= np.array([-1,1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,1])
            a += np.array([0,0,0,0,0,0,0,0,0,180,0,0,180,0,0,180,0,0])
            #print(a)
            a = a.reshape(-1,3)
            # print(a)
            # print('a:',a)
            return a
        # try:
        #     A_str, B_str, C_str = line.split(',')
        #     A, B, C = float(A_str), float(B_str), float(C_str)
        #     print(f"A={A}, B={B}, C={C}")

        # except ValueError:
        #     print("Formato inesperado:", line)

