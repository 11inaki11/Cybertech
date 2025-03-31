from machine import Pin, PWM
import utime
import machine

# Pines (los mismos que usabas)
PWM_PIN_D = 21
IN1_D = 9
IN2_D = 10
PWM_PIN_I = 18
IN1_I = 7
IN2_I = 8
STBY = 17
ENC_I = 6
ENC_D = 5

# Parámetros PID
Kp = 2.0
Ki = 0.1
Kd = 0.05

# Configuración PWM
def init_pwm(pin):
    pwm = PWM(Pin(pin), freq=1000, duty=0)
    return pwm

# Clase Motor con control PID
class MotorPID:
    def __init__(self, pwm_pin, in1, in2):
        self.pwm = init_pwm(pwm_pin)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.duty = 0
        self.rpm = 0
        self.setpoint = 0
        self.integral = 0
        self.last_error = 0

    def forward(self, duty):
        self.in1.value(1)
        self.in2.value(0)
        self.pwm.duty(int(min(max(duty, 0), 1023)))

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

    def update_pid(self, dt_ms):
        error = self.setpoint - self.rpm
        self.integral += error * dt_ms / 1000
        derivative = (error - self.last_error) / (dt_ms / 1000)
        output = Kp * error + Ki * self.integral + Kd * derivative
        self.duty += output
        self.last_error = error
        self.forward(self.duty)

# Motores
motorD = MotorPID(PWM_PIN_D, IN1_D, IN2_D)
motorI = MotorPID(PWM_PIN_I, IN1_I, IN2_I)

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

while True:
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
        if abs(error_sync) > 1:
            # Ajuste fino al setpoint
            motorI.setpoint += 0.2 * error_sync
            motorD.setpoint -= 0.2 * error_sync
