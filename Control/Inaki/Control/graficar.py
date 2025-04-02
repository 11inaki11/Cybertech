
import serial
import matplotlib.pyplot as plt
from collections import deque

# Configura el puerto serie (ajusta según tu sistema)
ser = serial.Serial('/dev/tty.usbmodem101', 115200)

# Buffers
max_len = 200
vel_d = deque([0]*max_len, maxlen=max_len)
vel_i = deque([0]*max_len, maxlen=max_len)
target = deque([0]*max_len, maxlen=max_len)

# Gráfica en tiempo real
plt.ion()
fig, ax = plt.subplots()
line_d, = ax.plot(vel_d, label='Vel. Derecha')
line_i, = ax.plot(vel_i, label='Vel. Izquierda')
line_t, = ax.plot(target, label='Objetivo', linestyle='--')
ax.legend()
ax.set_ylim(0, 50)
ax.set_xlabel('Tiempo (muestras)')
ax.set_ylabel('Velocidad (pulsos/50ms)')

while True:
    try:
        line = ser.readline().decode().strip()
        parts = line.split(',')
        if len(parts) == 3:
            vd, vi, tgt = map(int, parts)
            vel_d.append(vd)
            vel_i.append(vi)
            target.append(tgt)

            line_d.set_ydata(vel_d)
            line_i.set_ydata(vel_i)
            line_t.set_ydata(target)
            plt.pause(0.001)
    except Exception as e:
        print("Error:", e)