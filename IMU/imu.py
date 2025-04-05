from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep_ms

# Dirección del MPU6050 y registro de encendido
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43  # Giroscopio eje X

# Inicializa I2C con tus pines
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=400000)
i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')  # Despertar el MPU6050

# Función para leer registros de 16 bits con signo
def read_word(addr):
    high = i2c.readfrom_mem(MPU_ADDR, addr, 1)[0]
    low = i2c.readfrom_mem(MPU_ADDR, addr + 1, 1)[0]
    value = (high << 8) | low
    return value - 65536 if value > 32767 else value

# ------------------- CALIBRACIÓN DEL OFFSET -------------------
print("===================================")
print("🔧 Calibrando giroscopio en eje X...")
print("🛑 No mover el robot durante 1 segundo.")
print("===================================")

samples = 100
gyro_x_offset = 0.0
for _ in range(samples):
    gx = read_word(GYRO_XOUT_H)
    gyro_x_offset += gx / 131.0
    sleep_ms(10)
gyro_x_offset /= samples

print("✅ Calibración completada.")
print("Offset calculado en eje X: {:.3f} °/s".format(gyro_x_offset))
print("🟢 Iniciando medición de orientación...\n")

# ------------------- CÁLCULO DE ÁNGULO -------------------
orientation_x = 0.0
last_time = ticks_ms()

while True:
    # Leer velocidad angular en eje X y compensar offset
    gx = read_word(GYRO_XOUT_H)
    gyro_x = gx / 131.0 - gyro_x_offset  # °/s reales

    # Tiempo transcurrido desde la última lectura
    now = ticks_ms()
    dt = ticks_diff(now, last_time) / 1000.0  # segundos
    last_time = now

    # Integrar para obtener ángulo acumulado
    orientation_x += gyro_x * dt

    # Normalizar a rango 0–360°
    orientation_x = (orientation_x + 360) % 360

    # Mostrar orientación acumulada
    print("Orientación eje X (grados): {:.2f}".format(orientation_x))
    sleep_ms(50)
