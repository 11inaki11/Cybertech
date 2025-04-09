from machine import Pin, I2C, PWM, ADC
from time import ticks_ms, ticks_diff, sleep_ms
import math
import machine

# Constantes
FORWARD = 1
BACKWARD = 0
CLOCKWISE = 1
COUNTERCLOCKWISE = 0
PULSOS_POR_VUELTA = 960
DIAMETRO = 6.5  # cm
PERIMETRO = math.pi * DIAMETRO
DIST_POR_PULSO = PERIMETRO/ PULSOS_POR_VUELTA
d = 16.8 # Tamaño de cada celda
k = 3   # Umbral de pared
n = 64  # Número total de celdas del laberinto (se asume laberinto cuadrado)

# ---------------- SETUP DE PINES ----------------
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

# ------------------- CONFIGURACIÓN IR -------------------
# Pines IR
irI = ADC(Pin(1))
irC = ADC(Pin(2))
irD = ADC(Pin(3))
irI.atten(ADC.ATTN_11DB)
irC.atten(ADC.ATTN_11DB)
irD.atten(ADC.ATTN_11DB)
irI.width(ADC.WIDTH_12BIT)
irC.width(ADC.WIDTH_12BIT)
irD.width(ADC.WIDTH_12BIT)
# Función para leer voltaje
dist_izq = 0
dist_cen = 0
dist_der = 0


def leer_voltaje(ir):
    raw = ir.read()
    return raw * (3.3 / 4095)
# Función para convertir voltaje a distancia


def voltaje_a_distancia_f(v):
    if v < 0.3:
        return 100
    if v > 2.2:
        return 0
    return 4.9039 / (v - 0.0808)
# Función para leer distancia del IR frontal

def voltaje_a_distancia_d(v):
    if v < 0.3:
        return 100
    if v > 2.2:
        return 0
    return 4.5115 / (v - 0.0543)
# Función para leer distancia del IR lateral derecho

def voltaje_a_distancia_i(v):
    if v < 0.3:
        return 100
    if v > 2.2:
        return 0
    return 4.6221 / (v - 0.0563)
# Función para leer distancia del IR lateral izquierdo

def leer_distancia(ir):
    volt = leer_voltaje(ir)
    if ir is irC:
        distancia = voltaje_a_distancia_f(volt)
    elif ir is irD:
        distancia = voltaje_a_distancia_d(volt)
    elif ir is irI:
        distancia = voltaje_a_distancia_i(volt)
    else:
        distancia = None  # o podrías lanzar un error si el sensor no es válido
    return distancia

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
    duty = velocidad * 1023 / 100
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

def avance(velocidad, motorD, motorI):
    motorD.start()
    duty = velocidad * 1023 / 100
    motorD.duty = duty
    motorI.duty = duty
    motorI.move(FORWARD)
    motorD.move(FORWARD)

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
theta = 0.0
last_time = ticks_ms()
last_time_w = ticks_ms()
sample_time = 5
girando = 0  # Bandera para indicar si el robot ha girado mÃ¡s de 90Â°
completo = 0  # Bandera para indicar si el giro ha sido completado
completo1 = 0
completo2 = 0
# ------------------- CÃLCULO DE POSICIÓN -------------------
prev_pulsos_i = 0
prev_pulsos_d = 0
x = 0.0
y = 0.0
moviendo = 0 # Bandera para indicar si el robot se esta trasladando
moviendo_anterior = 0 # Bandera para indicar si en la última iteración del bucle el robot se estaba trasladando.
# Función para actualizar la posición
def actualizar_posicion(x, y, orientacion, delta_dist):
    # Convertir grados a radianes
    theta_rad = math.radians(orientacion)

    # Calcular nuevas coordenadas
    dy = delta_dist * math.cos(theta_rad)
    dx = delta_dist * math.sin(theta_rad)

    x += dx
    y += dy

    return x, y
# ------------------- MAQUINA DE ESTADOS -------------------
INICIO="INICIO" #Estado inicial, preparandose para empezar
GIRANDO="GIRANDO" #Listo para girar en el sitio
MOVIENDO="MOVIENDO" #Listo para moverse (trasladarse)
ANALIZAR="ANALIZAR" #Listo para analizar la nueva casilla
BAKTRACKING="BACKTRACKING" #Listo para retroceder
FINAL="FINAL" #Estado final, exploración terminada
RESOLVER="RESOLVER" #Estado final, exploración terminada listo para resolver el laberinto

# ------------------- CLASE MICROMOUSE MAPPER-------------------
delta_direccion = 0 #Variable que indica el giro que tiene que hacer el robot para ir a la nueva posición
delta_x = 0 #Variable que indica la diferencia de posición en x
delta_y = 0 #Variable que indica la diferencia de posición en y
mov_x=0 #Indica si me estoy desplazando en x durante la fase de traslación
mov_y=0 #Indica si me estoy desplazando en y durante la fase de traslación
backrtacking=0 #Indica si el robot está en fase de backtracking
backtracking_completado=0 #Indica si el robot ha completado la fase de backtracking
giros_bt = [] #Lista que almacena los giros que se deben realizar durante la fase de backtracking
movimientos_bt = [] #Lista que almacena los movimientos que se deben realizar durante la fase de backtracking
paso_actual = 0 #Variable que indica el paso actual del backtracking
backtracking_completado = False #Indica si el backtracking ha sido completado
guardado_final = False  # Bandera que indica si el camino final ha sido guardado


orientacion_objetivo = 0 #Variable que indica la orientación objetivo del robot para su siguiente movimiento
distancia_objetivo = 0 #Variable que indica la distancia objetivo del robot para su siguiente movimiento
angulo_rotacion=0 #Variable que indica el ángulo que se debe rotar para alcanzar orientación objetivo

# Clase para el mapeo del laberinto
class MicromouseMapper:
    def __init__(self, d, k, num_casillas_total):
        self.d = d  # tamaño de cada casilla
        self.k = k  # umbral para detectar pared
        self.current_position = (d / 2, d / 2)
        self.current_distance = 0
        self.visited = set()
        self.stack = [self.current_position]
        self.graph = {}
        self.graph[self.current_position] = {
            'distancia': 0,
            'hijo_dcha': False,
            'hijo_frente': False,
            'hijo_izq': False,
            'explorado': True
        }

        self.num_casillas_total = num_casillas_total
        self.lado = int(num_casillas_total ** 0.5)

        # Centro del laberinto (2x2 casillas centrales)
        offset = d / 2
        x1 = (self.lado // 2 - 1) * d + offset
        x2 = (self.lado // 2) * d + offset
        y1 = (self.lado // 2 - 1) * d + offset
        y2 = (self.lado // 2) * d + offset
        self.centro = ((x1 + x2) / 2, (y1 + y2) / 2)
        self.celdas_centro = [(x1, y1), (x1, y2), (x2, y1), (x2, y2)]

    def update_map(self, ur_front, ur_right, ur_left, orientation):
        pos = self.current_position
        if pos in self.visited:
            return

        can_go_front = ur_front > self.k
        can_go_right = ur_right > self.k
        can_go_left = ur_left > self.k

        self.graph[pos].update({
            'hijo_frente': can_go_front,
            'hijo_dcha': can_go_right,
            'hijo_izq': can_go_left,
            'explorado': True
        })

        self.add_unexplored_nodes(pos, can_go_front, can_go_right, can_go_left, orientation)
        self.visited.add(pos)

    def add_unexplored_nodes(self, position, f, r, l, orientation):
        for move, can_go in [('recto', f), ('derecha', r), ('izquierda', l)]:
            if can_go:
                new_pos = self.get_new_pos(position, move, orientation)
                if new_pos not in self.graph:
                    self.graph[new_pos] = {
                        'distancia': None,
                        'explorado': False,
                        'hijo_dcha': False,
                        'hijo_frente': False,
                        'hijo_izq': False
                    }

    def get_new_pos(self, position, direction, orientation):
        x, y = position
        d = self.d
        if orientation == 'N':
            if direction == 'recto': return (x, y + d)
            if direction == 'derecha': return (x + d, y)
            if direction == 'izquierda': return (x - d, y)
        elif orientation == 'S':
            if direction == 'recto': return (x, y - d)
            if direction == 'derecha': return (x - d, y)
            if direction == 'izquierda': return (x + d, y)
        elif orientation == 'E':
            if direction == 'recto': return (x + d, y)
            if direction == 'derecha': return (x, y - d)
            if direction == 'izquierda': return (x, y + d)
        elif orientation == 'O':
            if direction == 'recto': return (x - d, y)
            if direction == 'derecha': return (x, y + d)
            if direction == 'izquierda': return (x, y - d)

    def calcular_angulo_hacia_centro(self, origen):
        dx = self.centro[0] - origen[0]
        dy = self.centro[1] - origen[1]
        angulo_rad = math.atan2(dx, dy)
        return math.degrees(angulo_rad) % 360

    def distancia_angular(self, a, b):
        diff = abs(a - b) % 360
        return min(diff, 360 - diff)

    def elegir_direccion(self, orientation):
        node = self.graph[self.current_position]
        posibles_direcciones = []

        for nombre, clave in [('frente', 'recto'), ('dcha', 'derecha'), ('izq', 'izquierda')]:
            if node.get(f'hijo_{nombre}', False):
                destino = self.get_new_pos(self.current_position, clave, orientation)
                if not self.graph.get(destino, {}).get('explorado', False):
                    posibles_direcciones.append((clave, nombre, destino))

        if not posibles_direcciones:
            return 'backtrack'

        angulo_objetivo = self.calcular_angulo_hacia_centro(self.current_position)

        def angulo_direccion(destino):
            dx = destino[0] - self.current_position[0]
            dy = destino[1] - self.current_position[1]
            return math.degrees(math.atan2(dx, dy)) % 360

        mejores_opciones = sorted(
            posibles_direcciones,
            key=lambda x: (self.distancia_angular(angulo_direccion(x[2]), angulo_objetivo),['frente', 'dcha', 'izq'].index(x[1]))
        )

        return mejores_opciones[0][0]

    def calcular_camino_entre(self, inicio, fin):
        i1 = self.stack.index(inicio)
        i2 = self.stack.index(fin)
        if i1 < i2:
            return self.stack[i1:i2+1]
        else:
            return list(reversed(self.stack[i2:i1+1]))

    def opuesta_a(self, orientacion):
        mapa = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}
        return mapa.get(orientacion, 'N')

    def avanzar(self, direccion, orientation):
        if direccion == 'backtrack':
            destino = self.buscar_nodo_no_explorado()
            if destino:
                print(f"🔙 Backtracking hacia: {destino}")
                camino = self.calcular_camino_entre(self.current_position, destino)
                self.mostrar_giros_para_camino(camino, 'N')
                self.camino_final, self.giros_final = self.obtener_camino_y_giros(camino)
                self.current_position = destino
                self.current_distance = self.graph[destino]['distancia']
                return False

        else:
            nueva_pos = self.get_new_pos(self.current_position, direccion, orientation)
            self.graph[self.current_position][f"hijo_{direccion}"] = False
            self.current_position = nueva_pos
            self.stack.append(nueva_pos)
            padre = self.stack[-2]
            self.graph[nueva_pos]['distancia'] = self.graph[padre]['distancia'] + self.d
            self.current_distance = self.graph[nueva_pos]['distancia']

        if self.current_position in self.celdas_centro:
            print("🎯 ¡Centro alcanzado!")
            self.camino_final, self.giros_final = self.obtener_camino_y_giros(self.stack.copy())
            return True

        return False

    def mostrar_giros_para_camino(self, camino, orientacion_inicial):
        print("- derecha")
        print("- derecha")
        orientacion_actual = self.opuesta_a(orientacion_inicial)

        for i in range(1, len(camino)):
            actual = camino[i - 1]
            siguiente = camino[i]
            dx = siguiente[0] - actual[0]
            dy = siguiente[1] - actual[1]
            if dx == self.d: orientacion_obj = 'E'
            elif dx == -self.d: orientacion_obj = 'O'
            elif dy == self.d: orientacion_obj = 'N'
            elif dy == -self.d: orientacion_obj = 'S'
            else: continue
            giro = self.calcular_giro(orientacion_actual, orientacion_obj)
            print(f"- {giro}")
            print("- avanza")
            orientacion_actual = orientacion_obj

    def obtener_camino_y_giros(self, camino):
        movimientos = []
        giros = []
        orientacion_actual = 'N'

        for i in range(1, len(camino)):
            x1, y1 = camino[i - 1]
            x2, y2 = camino[i]
            dx = x2 - x1
            dy = y2 - y1
            if dx == self.d: direccion = 'E'
            elif dx == -self.d: direccion = 'O'
            elif dy == self.d: direccion = 'N'
            elif dy == -self.d: direccion = 'S'
            else: direccion = '?'
            movimientos.append((x2, y2))
            giros.append(self.calcular_giro(orientacion_actual, direccion))
            orientacion_actual = direccion

        return movimientos, giros

    def calcular_giro(self, desde, hacia):
        orden = ['N', 'E', 'S', 'O']
        i_desde = orden.index(desde)
        i_hacia = orden.index(hacia)
        diff = (i_hacia - i_desde) % 4
        if diff == 0: return 'recto'
        elif diff == 1: return 'derecha'
        elif diff == 2: return 'u'
        elif diff == 3: return 'izquierda'

    def buscar_nodo_no_explorado(self):
        for node in reversed(self.stack):
            datos = self.graph[node]
            for o in ['N', 'S', 'E', 'O']:
                for clave, tipo in [('frente', 'recto'), ('dcha', 'derecha'), ('izq', 'izquierda')]:
                    if datos.get(f'hijo_{clave}'):
                        hijo = self.get_new_pos(node, tipo, o)
                        if self.graph.get(hijo) and not self.graph[hijo].get('explorado'):
                            return node
        return None

def grados_a_orientacion(grados):
    grados = grados % 360  # Asegura que esté en [0, 360)
    if grados < 45 or grados >= 315:
        return 'N'
    elif 45 <= grados < 135:
        return 'E'
    elif 135 <= grados < 225:
        return 'S'
    elif 225 <= grados < 315:
        return 'O'
# ------------------- BUCLE PRINCIPAL -------------------
mapper = MicromouseMapper(d, k, n) #Inicio el micromouse mapper
estado_actual= INICIO # Estado inicial listo para analizar la casilla inicial

while True:
    # Leer velocidad angular en eje X y compensar offset
    now = ticks_ms()
    """
    if switch.value() == 1 and girando == 0 and completo == 0:
        # Iniciar giro en sentido horario a 50% de velocidad
        giro(CLOCKWISE, 15, motorD, motorI)
        girando = 1
        print("Giro iniciado. OrientaciÃ³n X: {:.2f}Â°".format(theta))
    """
    # aqui se leen todos los sensores
    if ticks_diff(now, last_time) >= sample_time:
        gx = read_word(GYRO_XOUT_H)
        gyro_x = gx / 131.0 - gyro_x_offset  # Â°/s reales

        # Tiempo transcurrido desde la Ãºltima lectura
        now = ticks_ms()
        dt = ticks_diff(now, last_time) / 1000.0  # segundos
        last_time = now

        # Integrar para obtener Ã¡ngulo acumulado
        theta -= gyro_x * dt

        # Lectura de los sensores IR
        # Leer sensores IR
        dist_izq = leer_distancia(irI)
        dist_cen = leer_distancia(irC)
        dist_der = leer_distancia(irD)


        # Normalizar a rango 0â360Â°
        theta = (theta + 360) % 360

        # Detectar cambio de estado de movimiento, de rotación a traslación.
        if moviendo_anterior == 0 and moviendo == 1:
            # Entrando en modo "moviendo" → resetear referencia de pulsos
            prev_pulsos_i = pulsos_i
            prev_pulsos_d = pulsos_d

        # Guardar estado actual como referencia para la próxima iteración. Si he empezado a moverme se pondrá esta bandera a 1. Solo se pondrá a 0 si el robot viene de una rotación
        moviendo_anterior = moviendo

        #Si el robot está en fase de traslación actualizo sus coordenadas (x,y)
        if moviendo:    
            # Guardar pulsos actuales y calcular delta
            irq_state = machine.disable_irq()
            delta_i = pulsos_i - prev_pulsos_i
            delta_d = pulsos_d - prev_pulsos_d
            prev_pulsos_i = pulsos_i
            prev_pulsos_d = pulsos_d
            machine.enable_irq(irq_state)

            # Distancia por rueda (en cm)
            dist_i = delta_i * DIST_POR_PULSO
            dist_d = delta_d * DIST_POR_PULSO

            # Distancia recorrida media del robot
            delta_dist = (dist_i + dist_d) / 2

            # Actualizar posición
            x, y = actualizar_posicion(x, y, theta, delta_dist)


    if ticks_diff(now, last_time_w) >= 500:
        print("Posición actual → x: {:.2f} cm, y: {:.2f} cm".format(x, y))
        print("Ángulo actual: {:.2f}°".format(theta))
        # Convertir None a 0.0 (o a otro valor de aviso si prefieres)
        izq = dist_izq if dist_izq is not None else 0.0
        cen = dist_cen if dist_cen is not None else 0.0
        der = dist_der if dist_der is not None else 0.0
        
        print("IR Izq: {:.2f} cm | IR Cen: {:.2f} cm | IR Der: {:.2f} cm".format(izq, cen, der))

    """
    if girando == 1 and theta < 90 and ticks_diff(now, last_time_w) >= 100:
        print("OrientaciÃ³n X: {:.2f}Â°".format(theta))
        last_time_w = now

    if abs(0 - theta) > 88 and abs(0 - theta) < 350:
        motorD.stop()
        motorI.stop()
        girando = 0
        completo = 1
        print("Giro completado. OrientaciÃ³n X: {:.2f}Â°".format(theta))

    elif switch.value() == 0 and completo:
        motorD.full_stop()
        motorI.full_stop()
    """

    """
    if y>100:
        motorD.stop()
        motorI.stop()
        moviendo = 0
        print("Movimiento completado. OrientaciÃ³n X: {:.2f}Â°".format(theta))
        print("Ángulo actual: {:.2f}°".format(theta))        
    """

    #######  MODO  EXPLORACIÓN DEL LABERINTO  #######

    if switch.value() == 0:
        if estado_actual == INICIO:
            #TRANSICIONES
            print("🟡 Estado: INICIO → Esperando 5 segundos antes de comenzar...")
            sleep_ms(5000)
            estado_actual = ANALIZAR

            #ACCIONES
            moviendo = 0 # Pongo a cero la bandera que indica que el robot no se está trasladando.

        elif estado_actual == ANALIZAR:
            #ACCIONES
            orientation = grados_a_orientacion(theta) #Convierto el ángulo a orientación cardinal (N, S, E, O)
            mapper.update_map(dist_cen, dist_der, dist_izq, orientation) #Actualizo el mapa con las lecturas de los sensores IR

            direccion = mapper.elegir_direccion(orientation) #Obtengo la dirección a la que me quiero mover (recto, derecha, izquierda o backtrack)

            if direccion == 'recto':
                delta_direccion = 0
            elif direccion == 'derecha':
                delta_direccion = 90
            elif direccion == 'izquierda':
                delta_direccion = -90
            elif direccion == 'backtrack':
                backrtacking = 1

            print(f"➡️  Dirección elegida: {direccion.upper()}")

            terminado = mapper.avanzar(direccion, orientation)

            posicion_objetivo= mapper.current_position
            delta_x=posicion_objetivo[0] - x #Diferencia de posición en x
            delta_y=posicion_objetivo[1] - y #Diferencia de posición en y

            #TRANSICIONES
            if terminado:
                estado_actual = FINAL
            if delta_direccion != 0:
                estado_actual = GIRANDO
            if delta_direccion == 0 and  (delta_x!= 0 or delta_y != 0):
                estado_actual = MOVIENDO
            if backrtacking == 1:
                estado_actual = BAKTRACKING
                backrtacking = 0

        elif estado_actual == FINAL:
            #ACCIONES
            print("\n✅ Exploración finalizada.")
            print("📌 Camino recorrido:")
            for paso in mapper.camino_final:
                print(paso)
            print("🔄 Giros:")
            print(mapper.giros_final)

            camino_resolucion = mapper.camino_final.copy() #Camino que soluciona el laberinto
            giros_resolucion = mapper.giros_final.copy() #Giros que se deben realizar para resolver el laberinto
            guardado_final = True
            
            #TRANSICIONES
            if guardado_final:
                guardado_final=0
                estado_actual=RESOLVER

        elif estado_actual == GIRANDO:  
            #ACCIONES
            if estado_anterior != GIRANDO:
                orientacion_objetivo = (theta + delta_direccion) % 360 #Orientación objetivo del robot (en grados)
                angulo_rotacion=((orientacion_objetivo - theta + 180) % 360) - 180 #Ángulo que debe rotar el robot (positivo sentido horario, negativo sentido antihorario)
                girando = 1
            if angulo_rotacion > 0:
                giro(CLOCKWISE, 15, motorD, motorI)
                diff = ((theta - orientacion_objetivo + 180) % 360) - 180
                if abs(diff) <= 2:
                    motorD.stop()
                    motorI.stop()
                    girando = 0
                    print("Giro completado. Orientación X: {:.2f}°".format(theta))
            
            if angulo_rotacion < 0:
                giro(COUNTERCLOCKWISE, 15, motorD, motorI)
                diff = ((theta - orientacion_objetivo + 180) % 360) - 180
                if abs(diff) <= 2:
                    motorD.stop()
                    motorI.stop()
                    girando = 0
                    print("Giro completado. Orientación X: {:.2f}°".format(theta))
                    
            #TRANSICIONES
            if girando==0: 
                estado_actual = MOVIENDO

        elif estado_actual == MOVIENDO:
            #ACCIONES
            if estado_anterior != MOVIENDO:
                if abs(delta_x) > abs(delta_y): 
                    mov_x=1
                    mov_y=0
                    x_objetivo=delta_x+x
                elif abs(delta_y) > abs(delta_x):
                    mov_x=0
                    mov_y=1
                    y_objetivo=delta_y+y
                moviendo = 1
            
            avance(15, motorD, motorI) #Avanza a 15% de velocidad
            if mov_x==1:
                diff=abs(x_objetivo-x)
                if diff <= 0.2:
                    motorD.stop()
                    motorI.stop()
                    moviendo = 0
                    mov_x=0
            if mov_y==1:
                diff=abs(y_objetivo-y)
                if diff <= 0.2:
                    motorD.stop()
                    motorI.stop()
                    moviendo = 0 
                    mov_y=0
            
            #TRANSICIONES
            if moviendo==0:
                estado_actual = ANALIZAR
                mov_y=0
                mov_x=0

        elif estado_actual == BAKTRACKING:
            #ACCIONES
            if estado_anterior != BAKTRACKING:
                giros_bt = mapper.giros_final
                movimientos_bt = mapper.camino_final
                paso_actual = 0
                backtracking_completado = 0
                orientacion_objetivo = None

            if paso_actual < len(giros_bt):
                giro_actual = giros_bt[paso_actual]
                movimiento_actual = movimientos_bt[paso_actual]

                # Determinar orientación objetivo si el giro no es recto
                if giro_actual == 'derecha':
                    delta_direccion = 90
                elif giro_actual == 'izquierda':
                    delta_direccion = -90
                elif giro_actual == 'u':
                    delta_direccion = 180
                else:
                    delta_direccion = 0

                # Ejecutar giro si es necesario
                if delta_direccion != 0 and orientacion_objetivo is None:
                    orientacion_objetivo = (theta + delta_direccion) % 360
                    angulo_rotacion = ((orientacion_objetivo - theta + 180) % 360) - 180
                    girando = True

                if girando:
                    if angulo_rotacion > 0:
                        giro(CLOCKWISE, 15, motorD, motorI)
                    else:
                        giro(COUNTERCLOCKWISE, 15, motorD, motorI)

                    diff = ((theta - orientacion_objetivo + 180) % 360) - 180
                    if abs(diff) <= 2:
                        motorD.stop()
                        motorI.stop()
                        girando = False
                        orientacion_objetivo = None  # Reset orientación

                # Si ya no gira y el giro fue 'recto', avanza
                if not girando and delta_direccion == 0:
                    if not moviendo:
                        moviendo = True
                        x_objetivo = movimiento_actual[0]
                        y_objetivo = movimiento_actual[1]
                        avance(15, motorD, motorI)

                    if moviendo:
                        if abs(x - x_objetivo) <= 0.2 and abs(y - y_objetivo) <= 0.2:
                            motorD.stop()
                            motorI.stop()
                            moviendo = False
                            paso_actual += 1
                elif not girando and delta_direccion != 0:
                    paso_actual += 1  # Una vez girado, pasar al siguiente paso
            
            else:
                # Una vez completados todos los pasos
                backtracking_completado = 1
            #TRANSICIONES
            if backtracking_completado == 1:
                estado_actual = ANALIZAR
                backtracking_completado = 0

        elif estado_actual == RESOLVER:
            a=0

    estado_anterior = estado_actual

    if switch.value() == 1:




