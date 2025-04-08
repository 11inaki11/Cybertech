from machine import Pin, I2C, PWM
from time import ticks_ms, ticks_diff, sleep_ms
import math
import machine

# Constantes
FORWARD = 1
BACKWARD = 0
CLOCKWISE = 1
COUNTERCLOCKWISE = 0
PULSOS_POR_VUELTA = 960

# ---------------- SETUP DE PINES ----------------
# Pines (los mismos que usabas)
PWM_PIN_D = 21
IN1_D = 10
IN2_D = 9
PWM_PIN_I = 18
IN1_I = 8
IN2_I = 7
STBY = 17
ENC_I = 6
ENC_D = 5
PIN_SW = 47

# ------------------ CONFIGURACIÃN PINES ------------------


def init_pwm(pin):
    pwm = PWM(Pin(pin), freq=1000, duty=0)
    return pwm


switch = Pin(PIN_SW, Pin.IN, Pin.PULL_UP)

# ---------------- CONFIGURACIÃN DE ENCODERS ----------------
# Pines encoder
encoderI = Pin(ENC_I, Pin.IN, Pin.PULL_UP)
encoderD = Pin(ENC_D, Pin.IN, Pin.PULL_UP)

# Contadores de pulsos (RPM)
pulsos_i = 0
pulsos_d = 0

# Interrupciones simples


def encoderI_cb(pin):
    global pulsos_i
    pulsos_i += 1


def encoderD_cb(pin):
    global pulsos_d
    pulsos_d += 1


encoderI.irq(trigger=Pin.IRQ_RISING, handler=encoderI_cb)
encoderD.irq(trigger=Pin.IRQ_RISING, handler=encoderD_cb)

# ------------------- CLASE MOTORES -------------------


class MotorPID:
    def __init__(self, pwm_pin, in1, in2, stby):
        self.pwm = init_pwm(pwm_pin)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.stby = Pin(stby, Pin.OUT)
        self.duty = 0
        self.rpm = 0
        self.setpoint = 0
        self.integral = 0
        self.last_error = 0

    def backward(self, duty):
        self.in1.value(0)
        self.in2.value(1)
        self.pwm.duty(int(min(max(duty, 0), 1023)))

    def forward(self, duty):
        self.in1.value(1)
        self.in2.value(0)
        self.pwm.duty(int(min(max(duty, 0), 1023)))

    def move(self, direction=FORWARD):
        '''Mover el motor en la direcciÃ³n especificada'''
        self.stby.value(1)
        if direction == FORWARD:
            self.forward(self.duty)
        else:
            self.backward(self.duty)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

    def full_stop(self):
        self.stop()
        self.stby.value(0)

    def start(self):
        self.stby.value(1)

    def update_pid(self, dt_ms):
        error = self.setpoint - self.rpm
        self.integral += error * dt_ms / 1000
        derivative = (error - self.last_error) / (dt_ms / 1000)
        output = Kp * error + Ki * self.integral + Kd * derivative
        self.duty += output
        self.last_error = error


# Motores
motorD = MotorPID(PWM_PIN_D, IN1_D, IN2_D, STBY)
motorI = MotorPID(PWM_PIN_I, IN1_I, IN2_I, STBY)


def giroCW(velocidad, motorD, motorI):
    duty = velocidad * 1023/100
    motorD.duty = duty
    motorI.duty = duty
    motorD.move(FORWARD)
    motorI.move(BACKWARD)


def giroCCW(velocidad, motorD, motorI):
    duty = velocidad * 1023 /100
    motorD.duty = duty
    motorI.duty = duty
    motorD.move(BACKWARD)
    motorI.move(FORWARD)


def giro(sentido, velocidad, motorD, motorI):
    motorD.start()
    if sentido == CLOCKWISE:
        giroCW(velocidad, motorD, motorI)
    elif sentido == COUNTERCLOCKWISE:
        giroCCW(velocidad, motorD, motorI)
      
# ------------------- CONFIGURACIÃN DEL MPU6050 -------------------
# DirecciÃ³n del MPU6050 y registro de encendido
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43  # Giroscopio eje X

# Inicializa I2C con tus pines
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=400000)
i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')  # Despertar el MPU6050

# FunciÃ³n para leer registros de 16 bits con signo


def read_word(addr):
    high = i2c.readfrom_mem(MPU_ADDR, addr, 1)[0]
    low = i2c.readfrom_mem(MPU_ADDR, addr + 1, 1)[0]
    value = (high << 8) | low
    return value - 65536 if value > 32767 else value


# ------------------- CALIBRACIÃN DEL OFFSET -------------------
print("===================================")
print("ð§ Calibrando giroscopio en eje X...")
print("ð No mover el robot durante 1 segundo.")
print("===================================")

samples = 100
gyro_x_offset = 0.0
for _ in range(samples):
    gx = read_word(GYRO_XOUT_H)
    gyro_x_offset += gx / 131.0
    sleep_ms(10)
gyro_x_offset /= samples

print("â CalibraciÃ³n completada.")
print("Offset calculado en eje X: {:.3f} Â°/s".format(gyro_x_offset))
print("ð¢ Iniciando mediciÃ³n de orientaciÃ³n...\n")

# ------------------- CÃLCULO DE ÃNGULO -------------------
orientation_x = 0.0
last_time = ticks_ms()
last_time_w = ticks_ms()
sample_time = 5
girando = 0  # Bandera para indicar si el robot ha girado mÃ¡s de 90Â°
completo = 0  # Bandera para indicar si el giro ha sido completado
completo1 = 0
completo2 = 0
# ------------------- BUCLE PRINCIPAL -------------------

while True:
    # Leer velocidad angular en eje X y compensar offset
    now = ticks_ms()
    if switch.value() == 1 and girando == 0 and completo == 0:
        # Iniciar giro en sentido horario a 50% de velocidad
        giro(CLOCKWISE, 15, motorD, motorI)
        girando = 1
        print("Giro iniciado. OrientaciÃ³n X: {:.2f}Â°".format(orientation_x))

    if ticks_diff(now, last_time) >= sample_time:
        gx = read_word(GYRO_XOUT_H)
        gyro_x = gx / 131.0 - gyro_x_offset  # Â°/s reales

        # Tiempo transcurrido desde la Ãºltima lectura
        now = ticks_ms()
        dt = ticks_diff(now, last_time) / 1000.0  # segundos
        last_time = now

        # Integrar para obtener Ã¡ngulo acumulado
        orientation_x += gyro_x * dt

        # Normalizar a rango 0â360Â°
        #orientation_x = (orientation_x + 360) % 360

    if girando == 1 and orientation_x < 90 and ticks_diff(now, last_time_w) >= 100:
        print("OrientaciÃ³n X: {:.2f}Â°".format(orientation_x))
        last_time_w = now

    if abs(0 - orientation_x) > 88 and abs(0 - orientation_x) < 350:
        motorD.stop()
        motorI.stop()
        girando = 0
        completo = 1
        print("Giro completado. OrientaciÃ³n X: {:.2f}Â°".format(orientation_x))

    elif switch.value() == 0 and completo:
        motorD.full_stop()
        motorI.full_stop()
