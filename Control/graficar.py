import serial
import matplotlib.pyplot as plt

# Configurar puerto serie (cambia "COM3" o "/dev/ttyUSB0" según corresponda)
ser = serial.Serial("COM4", 115200, timeout=1)

# Variables para la gráfica
rpm_values = []
filtered_values = []  # Lista para almacenar la media de cada 5 valores
window_size = 5  # Tamaño de la ventana para la media móvil

plt.ion()  # Modo interactivo

while True:
    try:
        data = ser.readline().decode().strip()  # Leer datos del ESP32
        if data:
            rpm = float(data)
            rpm_values.append(rpm)

            # Aplicar filtro de media móvil cada 5 valores
            if len(rpm_values) >= window_size:
                avg_rpm = sum(rpm_values[-window_size:]) / window_size  # Calcular media
                filtered_values.append(avg_rpm)

            # Mostrar en la gráfica
            plt.clf()
            plt.plot(filtered_values, label="RPM (Media de 5)")
            plt.xlabel("Tiempo")
            plt.ylabel("Velocidad (RPM)")
            plt.legend()
            plt.pause(0.1)  # Pequeño delay para actualizar
    except KeyboardInterrupt:
        break

ser.close()
plt.ioff()
plt.show()
