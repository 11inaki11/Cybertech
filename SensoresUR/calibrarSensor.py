from machine import ADC, Pin
from time import sleep
import sys

# Configuración del ADC
adc = ADC(Pin(2))  # Cambia por tu pin
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)

# Leer voltaje del sensor
def leer_voltaje():
    raw = adc.read()
    return raw * (3.3 / 4095)

# Ruta del archivo CSV
archivo = 'datos_ir.csv'

# Si el archivo no existe, crea la cabecera
try:
    with open(archivo, 'r') as f:
        pass  # Ya existe
except OSError:
    with open(archivo, 'w') as f:
        f.write('distancia_cm,voltaje_v\n')

# Bucle principal
while True:
    try:
        # Solicitar distancia real
        distancia = input("Introduce la distancia real en cm (o escribe 'fin' para salir): ")
        if distancia.lower() == 'fin':
            print("Finalizando...")
            break

        distancia = float(distancia)
        print("Midiento 100 veces a {:.2f} cm...".format(distancia))

        mediciones = []
        for i in range(100):
            volt = leer_voltaje()
            mediciones.append(volt)
            print("Medición {:3d}: {:.4f} V".format(i+1, volt))
            sleep(0.05)

        # Guardar en CSV
        with open(archivo, 'a') as f:
            for v in mediciones:
                f.write("{:.2f},{:.4f}\n".format(distancia, v))

        print("Guardado 100 muestras para {:.2f} cm.\n".format(distancia))

    except Exception as e:
        print("Error:", e)
