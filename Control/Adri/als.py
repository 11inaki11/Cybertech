from machine import Pin, PWM
import utime
import machine

# Configuración de pines (manteniendo tus definiciones)
PWM_PIN_D = 21  # Motor derecho
IN1_D = 9
IN2_D = 10
PWM_PIN_I = 18  # Motor izquierdo
IN1_I = 7
IN2_I = 8
STBY = 17       # Standby (activar ambos motores)
ENC_I = 6       # Encoder izquierdo
ENC_D = 5       # Encoder derecho

## Parámetros optimizados ##
TARGET_RPM = 60             # Velocidad objetivo
PULSES_PER_REV = 960        # Pulsos por vuelta del encoder
SAMPLE_TIME_MS = 50         # Tiempo de muestreo (reducido para mayor precisión)
MIN_DUTY = 300              # Duty cycle mínimo para vencer rozamiento
MAX_DUTY = 900              # Duty cycle máximo para evitar saturación

# Configuración PID optimizada
KP = 1.5         # Ganancia proporcional (respuesta rápida)
KI = 0.4       # Ganancia integral (elimina error estacionario)
KD = 0.4       # Ganancia derivativa (reduce oscilaciones)
SYNC_GAIN = 0.25  # Ganancia para sincronización entre motores

# Filtrado más agresivo:
filter_factor = 0.8  # Para el cálculo de filtered_rpm

class MotorController:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWM(Pin(pwm_pin), freq=2000, duty=0)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.duty = 0
        self.rpm = 0
        self.filtered_rpm = 0
        self.setpoint = 0
        self.last_error = 0
        self.integral = 0
        
    def move(self, duty):
        duty = int(min(max(duty, MIN_DUTY), MAX_DUTY))
        
        # Aplicar factor de corrección calibrado
        if self == motorI:  # Si es motor izquierdo
            duty = int(duty * LEFT_MOTOR_FACTOR)
        
        self.duty = duty
        if duty > 0:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(1)
            self.in2.value(0)
        self.pwm.duty(abs(duty))
    
    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)
        self.duty = 0
        
    def update_pid(self, dt_sec):
        """Control PID con anti-windup y filtrado"""
        error = self.setpoint - self.filtered_rpm
        
        # Término proporcional
        P = KP * error
        
        # Término integral con anti-windup
        self.integral += error * dt_sec
        I = KI * self.integral
        
        # Término derivativo
        D = KD * (error - self.last_error) / dt_sec if dt_sec > 0 else 0
        self.last_error = error
        
        # Calcular nueva salida
        pid_output = P + I + D
        new_duty = self.duty + pid_output
        
        # Aplicar movimiento con nuevo duty
        self.move(new_duty)

# Inicialización de motores
motorD = MotorController(PWM_PIN_D, IN1_D, IN2_D)
motorI = MotorController(PWM_PIN_I, IN1_I, IN2_I)

# Habilitar drivers
standby = Pin(STBY, Pin.OUT)
standby.value(1)

# Configuración de encoders
encoderI = Pin(ENC_I, Pin.IN, Pin.PULL_UP)
encoderD = Pin(ENC_D, Pin.IN, Pin.PULL_UP)

# Variables para medición de velocidad
pulse_count_i = 0
pulse_count_d = 0
last_pulse_i = 0
last_pulse_d = 0

def encoder_isr(pin):
    """Interrupt Service Routine para encoders"""
    global pulse_count_i, pulse_count_d
    if pin == encoderI:
        pulse_count_i += 1
    else:
        pulse_count_d += 1

# Configurar interrupciones
encoderI.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr)
encoderD.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr)

def sync_motors(rpm_left, rpm_right):
    """Sistema avanzado de sincronización"""
    error = rpm_left - rpm_right
    
    # Solo corregir si la diferencia es significativa
    if abs(error) > 0.5:
        adjustment = error * SYNC_GAIN
        motorI.setpoint = adjustment
        motorD.setpoint = adjustment
        
        # Mantener la velocidad promedio constante
        avg_speed = (motorI.setpoint + motorD.setpoint) / 2
        motorI.setpoint += avg_speed
        motorD.setpoint += avg_speed

# Bucle principal mejorado
last_time = utime.ticks_ms()
motorD.setpoint = TARGET_RPM
motorI.setpoint = TARGET_RPM

while True:
    current_time = utime.ticks_ms()
    elapsed = utime.ticks_diff(current_time, last_time)
    
    # CONTROL MOTORES
    if elapsed >= SAMPLE_TIME_MS:
        # Sección crítica para lectura de encoders
        irq_state = machine.disable_irq()
        delta_i = pulse_count_i - last_pulse_i
        delta_d = pulse_count_d - last_pulse_d
        last_pulse_i = pulse_count_i
        last_pulse_d = pulse_count_d
        machine.enable_irq(irq_state)
        
        # Calcular RPMs con filtrado
        dt_sec = elapsed / 1000.0
        motorI.rpm = calculate_rpm(delta_i, elapsed)
        motorD.rpm = calculate_rpm(delta_d, elapsed)

        # Ajusta estos factores según lo observado
        if motorI.rpm > motorD.rpm:
            LEFT_MOTOR_FACTOR = 0.90  # Reduce izquierda
        else:
            LEFT_MOTOR_FACTOR = 1.05  # Aumenta izquierda
  
        # Filtro paso bajo para las RPMs
        motorI.filtered_rpm = filter_factor * motorI.filtered_rpm + 0.3 * motorI.rpm
        motorD.filtered_rpm = filter_factor * motorD.filtered_rpm + 0.3 * motorD.rpm
        
        # Sincronización de motores
        sync_motors(motorI.filtered_rpm, motorD.filtered_rpm)
        
        # Actualizar controladores PID
        motorI.update_pid(dt_sec)
        motorD.update_pid(dt_sec)
        
        last_time = current_time