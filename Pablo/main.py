from machine import Pin, PWM
import time

# Definición de los pines
switch_func_pin = 47
driver_pin_standby = 17
motor_A_pin_direction_fwd = 8
motor_A_pin_direction_bwd = 7
motor_A_pin_encoder = 6
motor_A_pin_pwm = 18
motor_B_pin_direction_fwd = 10
motor_B_pin_direction_bwd = 9
motor_B_pin_encoder = 5
motor_B_pin_pwm = 21

# Configuración de los pines
switch_func = Pin(switch_func_pin, Pin.IN, Pin.PULL_UP)
driver_standby = Pin(driver_pin_standby, Pin.OUT)
motor_A_direction_fwd = Pin(motor_A_pin_direction_fwd, Pin.OUT)
motor_A_direction_bwd = Pin(motor_A_pin_direction_bwd, Pin.OUT)
motor_A_encoder = Pin(motor_A_pin_encoder, Pin.IN, Pin.PULL_UP)
motor_A_pwm = PWM(Pin(motor_A_pin_pwm))
motor_B_direction_fwd = Pin(motor_B_pin_direction_fwd, Pin.OUT)
motor_B_direction_bwd = Pin(motor_B_pin_direction_bwd, Pin.OUT)
motor_B_encoder = Pin(motor_B_pin_encoder, Pin.IN, Pin.PULL_UP)
motor_B_pwm = PWM(Pin(motor_B_pin_pwm))

# Funciones para establecer las velocidades de los motores A y B


def set_motor_A_speed(speed_percent):
    # El duty cycle va de 0 (detenido) a 1023 (máxima velocidad para PWM de 10 bits en ESP32)
    duty_cycle = int(speed_percent * 1023 / 100)
    motor_A_pwm.duty(duty_cycle)


def set_motor_B_speed(speed_percent):
    # El duty cycle va de 0 (detenido) a 1023 (máxima velocidad para PWM de 10 bits en ESP32)
    duty_cycle = int(speed_percent * 1023 / 100)
    motor_B_pwm.duty(duty_cycle)

# Funciones para comportamiento frente a interrupcion de los encoders de los motores A y B


def encoder_A_isr(pin):
    global encoder_A_count
    encoder_A_count = encoder_A_count + 1


def encoder_B_isr(pin):
    global encoder_B_count
    encoder_B_count = encoder_B_count + 1


# Configuración de las interrupciones para los encoders
motor_A_encoder.irq(trigger=Pin.IRQ_RISING, handler=encoder_A_isr)
motor_B_encoder.irq(trigger=Pin.IRQ_RISING, handler=encoder_B_isr)

# Inicialización de las variables
encoder_A_count = 0
encoder_B_count = 0
motor_A_distancia = 0
motor_B_distancia = 0

# Código principal
if __name__ == '__main__':

    print("Comenzamos el movimiento de los motores A y B")
    set_motor_A_speed(0)  # Iniciar con el motor A detenido
    set_motor_B_speed(0)  # Iniciar con el motor B detenido
    driver_standby.on()  # Habilitar el driver de los motores
    print("Driver habilitado")
    time.sleep(1)
    print("Configurando movimiento hacia delante")
    set_motor_A_speed(50)  # Velocidad motor A
    set_motor_B_speed(50)  # Velocidad motor B
    time.sleep(1)
    print("Encendiendo motores A y B")
    motor_A_direction_fwd.on()
    motor_A_direction_bwd.off()
    motor_B_direction_fwd.on()
    motor_B_direction_bwd.off()

    while True:

        motor_A_distancia = (encoder_A_count/960)*2*3.1416*3.5
        motor_B_distancia = (encoder_B_count/960)*2*3.1416*3.5

        if ((motor_A_distancia >= 50) or (motor_B_distancia >= 50)):
            motor_A_direction_fwd.off()
            motor_B_direction_fwd.off()
            print("Distancia maxima alcanzada.\nMovimiento detenido.\n")
            print("El encoder del motor A ha leido: ", encoder_A_count)
            print("El encoder del motor B ha leido: ", encoder_B_count)
            print("Distancia recorrida por motor A: ", motor_A_distancia)
            print("Distancia recorrida por motor B: ", motor_B_distancia)
            break
