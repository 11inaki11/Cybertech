from machine import Pin, PWM
import machine
import utime


class Motor:
    def __init__(self, pwm_pin, in1, in2, stby):
        self.pwm = PWM(Pin(pwm_pin), freq=1000, duty=0)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.stby = Pin(stby, Pin.OUT)

    def movef(self, speed):
        self.in1.value(1)
        self.in2.value(0)
        self.pwm.duty(abs(speed))
        self.stby.value(1)

    def moveb(self, speed):
        self.in1.value(0)
        self.in2.value(1)
        self.pwm.duty(abs(speed))
        self.stby.value(1)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

    def full_stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)
        self.stby.value(0)


# Configuración de pines para el driver TB6612FNG
PWM_PIN_D = 21  # Pin PWM para velocidad (AIN1 o BIN1)
IN1_D = 9      # Dirección del motor (AIN2 o BIN2)
IN2_D = 10      # Dirección del motor (AIN1 o BIN1)
PWM_PIN_I = 18
IN1_I = 7
IN2_I = 8    # Dirección del motor D
STBY = 17     # Standby (activar el driver)

# Configuración de pines para el encoder
ENC_I = 6  # Encoder izquierdo
ENC_D = 7  # Encoder derecho

# Crear objetos para los motores
motorD = Motor(PWM_PIN_D, IN1_D, IN2_D, STBY)   # Motor derecho
motorI = Motor(PWM_PIN_I, IN1_I, IN2_I, STBY)   # Motor izquierdo

encoderI = Pin(ENC_I, Pin.IN, Pin.PULL_UP)
encoderD = Pin(ENC_D, Pin.IN, Pin.PULL_UP)

# Variables para el encoder
contadorI = 0
contadorD = 0
prevcontadorI = 0
prevcontadorD = 0
rpmI = 0
rpmD = 0
last_time = utime.ticks_ms()

# Interrupciones de los encoders


def callback_encoderI(pin):
    global contadorI
    contadorI += 1


def callback_encoderD(pin):
    global contadorD
    contadorD += 1


# Configurar interrupciones
encoderI.irq(trigger=Pin.IRQ_RISING, handler=callback_encoderI)
encoderD.irq(trigger=Pin.IRQ_RISING, handler=callback_encoderD)

motorI.movef(300)
flag = 0
flag2 = 2
while flag != 30000:
    current_time = utime.ticks_ms()
    flag += 1
    if (current_time - last_time) > 400:
        flag2 = 1

print("el valor de flag es", flag2)
motorI.full_stop()
