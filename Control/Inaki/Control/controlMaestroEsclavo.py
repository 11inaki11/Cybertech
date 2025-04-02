from machine import Pin, PWM, Timer
import time

# Pines
PWM_PIN_D = 21
IN1_D = 9
IN2_D = 10
ENC_D = 5

PWM_PIN_I = 18
IN1_I = 7
IN2_I = 8
ENC_I = 6

STBY = 17

# Configuración de pines
stby = Pin(STBY, Pin.OUT)
stby.value(1)

# PWM
FREQ = 1000
pwm_d = PWM(Pin(PWM_PIN_D), freq=FREQ, duty=0)
pwm_i = PWM(Pin(PWM_PIN_I), freq=FREQ, duty=0)

in1_d = Pin(IN1_D, Pin.OUT)
in2_d = Pin(IN2_D, Pin.OUT)
in1_i = Pin(IN1_I, Pin.OUT)
in2_i = Pin(IN2_I, Pin.OUT)

# Variables globales de encoder
count_d = 0
count_i = 0

# Encoder interrupts
def encoder_d_handler(pin):
    global count_d
    count_d += 1

def encoder_i_handler(pin):
    global count_i
    count_i += 1

encoder_d = Pin(ENC_D, Pin.IN)
encoder_i = Pin(ENC_I, Pin.IN)
encoder_d.irq(trigger=Pin.IRQ_RISING, handler=encoder_d_handler)
encoder_i.irq(trigger=Pin.IRQ_RISING, handler=encoder_i_handler)

# Clase PID
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def compute(self, setpoint, measured):
        error = setpoint - measured
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# PID por rueda
pid_d = PID(kp=1.2, ki=0.2, kd=0.1)   # Maestra
pid_i = PID(kp=1.5, ki=0.25, kd=0.12) # Esclava

# Función para aplicar PWM y dirección
def set_motor(pwm, in1, in2, speed):
    if speed > 0:
        in1.value(0)  # Cambiado para ir hacia adelante
        in2.value(1)
    elif speed < 0:
        in1.value(1)
        in2.value(0)
    else:
        in1.value(0)
        in2.value(0)
    pwm.duty(min(abs(int(speed)), 1023))

# Velocidad deseada
TARGET_SPEED_D = 20  # Pulsos por intervalo

# Bucle de control
def control_loop(timer):
    global count_d, count_i

    # Medir velocidades actuales
    speed_d = count_d
    speed_i = count_i
    count_d = 0
    count_i = 0

    # Rueda derecha: control normal a setpoint fijo
    output_d = pid_d.compute(TARGET_SPEED_D, speed_d)

    # Rueda izquierda: intenta igualar la velocidad de la derecha
    output_i = pid_i.compute(speed_d, speed_i)

    # Aplicar señales
    set_motor(pwm_d, in1_d, in2_d, output_d)
    set_motor(pwm_i, in1_i, in2_i, output_i)

    # Imprimir para depuración
    print("Vel D: {:2d} | PID D: {:6.1f} || Vel I: {:2d} | PID I: {:6.1f}".format(
        speed_d, output_d, speed_i, output_i))

# Temporizador de control
timer = Timer(0)
timer.init(period=50, mode=Timer.PERIODIC, callback=control_loop)
