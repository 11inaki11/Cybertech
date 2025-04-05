from machine import Pin, PWM
import utime
import machine
import math

# Constantes
FORWARD = 1
BACKWARD = 0

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

# Parámetros PID
Kp = 2.0
Ki = 0.1
Kd = 0.05

switch = Pin(PIN_SW, Pin.IN, Pin.PULL_UP)

# Configuración PWM


def init_pwm(pin):
    pwm = PWM(Pin(pin), freq=1000, duty=0)
    return pwm

# Clase Motor con control PID


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
        '''Mover el motor en la dirección especificada'''
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

# Control principal
PULSOS_POR_VUELTA = 960
sample_time = 50  # ms
motorD.setpoint = 60  # RPM objetivo
motorI.setpoint = 60

prev_pulsos_i = 0
prev_pulsos_d = 0
last_time = utime.ticks_ms()

if switch.value() == 0 and motorD.stby.value() == 1:
    motorD.full_stop()
    motorI.full_stop()

if switch.value() == 1 and motorD.stby.value() == 0:
    motorD.start()

while True:
    if switch.value() == 1:
        now = utime.ticks_ms()
        if utime.ticks_diff(now, last_time) >= sample_time:
            dt = utime.ticks_diff(now, last_time)
            last_time = now

            # Sección crítica mínima
            irq_return = machine.disable_irq()
            delta_i = pulsos_i - prev_pulsos_i
            delta_d = pulsos_d - prev_pulsos_d
            prev_pulsos_i = pulsos_i
            prev_pulsos_d = pulsos_d
            machine.enable_irq(irq_return)

            # Cálculo de RPM
            motorI.rpm = (delta_i * 60 * 1000) / (PULSOS_POR_VUELTA * dt)
            motorD.rpm = (delta_d * 60 * 1000) / (PULSOS_POR_VUELTA * dt)

            # Control PID
            motorI.update_pid(dt)
            motorD.update_pid(dt)

            # Sincronización: igualar velocidades si hay diferencias pequeñas

            error_sync = motorD.rpm - motorI.rpm
            """
            if abs(error_sync) > 1:
                # Ajuste fino al setpoint
                motorI.setpoint += math.copysign(5, error_sync)
                motorD.setpoint -= math.copysign(5, error_sync)
            """

            # Actualizar el PWM
            motorI.move(FORWARD)
            motorD.move(FORWARD)

            """
            if sensorIR[2] < sensorIR[1]:
                motorI.setpoint += 5
            elif sensorIR[2] > sensorIR[1]:
                motorD.setpoint -= 5
            """

    else:
        motorI.full_stop()
        motorD.full_stop()
