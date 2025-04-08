from machine import ADC, Pin
from time import sleep

adc = ADC(Pin(1))  # Cambia por tu pin
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)

def leer_voltaje():
    raw = adc.read()
    return raw * (3.3 / 4095)

def voltaje_a_distancia(v):
    if v < 0.3 or v > 2.2:
        return None
    return 4.6221 / (v - 0.0563)

while True:
    volt = leer_voltaje()
    distancia = voltaje_a_distancia(volt)
    if distancia:
        print("Voltaje: {:.2f} V | Distancia: {:.2f} cm".format(volt, distancia))
    else:
        print("Voltaje: {:.2f} V | Fuera de rango".format(volt))
    sleep(0.5)
