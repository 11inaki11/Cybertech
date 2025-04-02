import serial
import matplotlib.pyplot as plt

# Configura el puerto serie
ser = serial.Serial("/dev/tty.usbmodem101", 115200, timeout=1)

# Variables para la gráfica
vel_d = []
vel_i = []
target = []

filtered_d = []
filtered_i = []
filtered_t = []

window_size = 5  # Tamaño de la ventana para la media móvil

plt.ion()  # Modo interactivo

while True:
    try:
        data = ser.readline().decode().strip()
        if data:
            parts = data.split(',')
            if len(parts) == 3:
                d = float(parts[0])
                i = float(parts[1])
                t = float(parts[2])

                vel_d.append(d)
                vel_i.append(i)
                target.append(t)

                if len(vel_d) >= window_size:
                    avg_d = sum(vel_d[-window_size:]) / window_size
                    avg_i = sum(vel_i[-window_size:]) / window_size
                    avg_t = sum(target[-window_size:]) / window_size

                    filtered_d.append(avg_d)
                    filtered_i.append(avg_i)
                    filtered_t.append(avg_t)

                # Mostrar gráfica
                plt.clf()
                plt.plot(filtered_d, label="Vel. Derecha (media 5)")
                plt.plot(filtered_i, label="Vel. Izquierda (media 5)")
                plt.plot(filtered_t, label="Objetivo (media 5)", linestyle='--')
                plt.xlabel("Tiempo")
                plt.ylabel("Velocidad (pulsos/50ms)")
                plt.legend()
                plt.pause(0.1)

    except KeyboardInterrupt:
        break

ser.close()
plt.ioff()
plt.show()
