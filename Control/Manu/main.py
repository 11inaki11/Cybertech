from machine import Pin, PWM
import utime
import machine

class Motor:
    '''Clase para controlar un motor con un driver L298N'''

    def __init__(self, pwm_pin, in1, in2, stby, setpoint=0):
        self.pwm = PWM(Pin(pwm_pin), freq=1000, duty=0)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.stby = Pin(stby, Pin.OUT)

        self.setpoint = setpoint  # RPM objetivo
        self.rpm = 0              # RPM actual medida
        self.rpm_reg = [0, 0, 0, 0, 0]  # Registro de RPM (últimos 5 valores)
        self.rpm_filtrada = 0      # Valor filtrado de RPM
        self.duty_cycle = 0        # Duty cycle actual

    def movef(self, speed):
        '''Mover hacia adelante'''
        self.in1.value(1)
        self.in2.value(0)
        self.pwm.duty(abs(speed))
        self.stby.value(1)

    def moveb(self, speed):  # Mover hacia atrás
        '''Mover hacia atrás'''
        self.in1.value(0)
        self.in2.value(1)
        self.pwm.duty(abs(speed))
        self.stby.value(1)

    def stop(self):
        '''Parar el motor'''
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

    def full_stop(self):
        '''Parar el motor y desactivar el driver'''
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)
        self.stby.value(0)

    def control(self):
        '''Controlar la velocidad del motor'''
        # Actualizar registro de RPM
        if len(self.rpm_reg) >= 5:
            self.rpm_reg.pop(4)  # Elimina el elemento en la posición 5
        # Insertar la nueva lectura en la primera posición
        self.rpm_reg.insert(0, self.rpm)

        # Aplicar filtro si hay suficientes datos
        if all(self.rpm_reg[1:]):
            self.rpm_filtrada = (self.rpm_reg[0] * 0.5 +
                                 self.rpm_reg[1] * 0.25 +
                                 self.rpm_reg[2] * 0.15 +
                                 self.rpm_reg[3] * 0.08 +
                                 self.rpm_reg[4] * 0.02)
        else:
            self.rpm_filtrada = self.rpm_reg[0]

        # Calcular error y actualizar duty cycle
        error = self.rpm_filtrada / self.setpoint
        self.duty_cycle = regulador(error, self.duty_cycle)

    def move(self, direction = 1):
        '''Mover el motor en la dirección especificada'''
        # Aplicar al motor
        if direction == 1:
            self.movef(self.duty_cycle)
        else:
            self.moveb(self.duty_cycle)

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
ENC_D = 5  # Encoder derecho

# Crear objetos para los motores
motorD = Motor(PWM_PIN_D, IN1_D, IN2_D, STBY)   # Motor derecho
motorI = Motor(PWM_PIN_I, IN1_I, IN2_I, STBY)   # Motor izquierdo

encoderI = Pin(ENC_I, Pin.IN, Pin.PULL_UP)
encoderD = Pin(ENC_D, Pin.IN, Pin.PULL_UP)

# Variables para el encoder
contador_i = 0
contador_d = 0
prev_contador_i = 0
prev_contador_d = 0
rpmI = 0
rpmD = 0

# Variables para el control PID
sample_time_v = 10  # ms

last_time_v = utime.ticks_ms()
last_time_w = utime.ticks_ms()

# Interrupciones de los encoders
def callback_encoderI(pin):
    '''Callback para el encoder izquierdo'''
    global contador_i
    contador_i += 1

def callback_encoderD(pin):
    '''Callback para el encoder derecho'''
    global contador_d
    contador_d += 1


# Configurar interrupciones
encoderI.irq(trigger=Pin.IRQ_RISING, handler=callback_encoderI)
encoderD.irq(trigger=Pin.IRQ_RISING, handler=callback_encoderD)

time.sleep(1)  # Esperar un segundo para estabilizar el sistema

while (1):
    if utime.ticks_diff(utime.ticks_ms(), last_time_v) > sample_time_v:
        disable_out = machine.disable_irq()  # se deshabilitan interrupciones
        # -- sección crítica --
        delta_i = contador_i - prev_contador_i
        delta_d = contador_d - prev_contador_d
        prev_contador_i = contador_i
        prev_contador_d = contador_d
        # -- sección crítica --
        machine.enable_irq(disable_out)  # vuelven a habilitarse interrupciones
        motorI.rpm = (delta_i*60 * 1000) / (960 * sample_time_v)
        motorD.rpm = (delta_d*60 * 1000) / (960 * sample_time_v)
        last_time_v = utime.ticks_ms()
        motorD.control()
        motorI.control()

        error_sync = motorD.rpm - motorI.rpm
        if abs(error_sync) > 1:
            # Ajuste fino al setpoint
            motorI.setpoint += 0.2 * error_sync
            motorD.setpoint -= 0.2 * error_sync

    if utime.ticks_diff(utime.ticks_ms(), last_time_w)> 100:
        print(motorI.rpm, motorD.rpm)
