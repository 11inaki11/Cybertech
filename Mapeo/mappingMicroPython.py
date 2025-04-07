import math

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
            key=lambda x: (self.distancia_angular(angulo_direccion(x[2]), angulo_objetivo),
                           ['frente', 'dcha', 'izq'].index(x[1]))
        )

        return mejores_opciones[0][0]

    def avanzar(self, direccion, orientation):
        if direccion == 'backtrack':
            destino = self.buscar_nodo_no_explorado()
            if destino:
                self.current_position = destino
                self.current_distance = self.graph[destino]['distancia']
            else:
                print("✅ Exploración completa.")
                return True
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

def main():
    d = 18  # Tamaño de cada celda
    k = 9   # Umbral de pared
    n = 64  # Número total de celdas del laberinto

    mapper = MicromouseMapper(d, k, n)

    while True:
        print(f"\n📍 Posición actual: {mapper.current_position} - Distancia: {mapper.current_distance}")

        try:
            ur_front = float(input("Distancia sensor frontal: "))
            ur_right = float(input("Distancia sensor derecho: "))
            ur_left = float(input("Distancia sensor izquierdo: "))
            angulo = float(input("Ángulo de orientación (grados): "))
            orientation = grados_a_orientacion(angulo)

            mapper.update_map(ur_front, ur_right, ur_left, orientation)
            direccion = mapper.elegir_direccion(orientation)
            print(f"➡️  Dirección elegida: {direccion.upper()}")

            terminado = mapper.avanzar(direccion, orientation)
            if terminado:
                print("\n✅ Exploración finalizada.")
                print("📌 Camino recorrido:")
                for paso in mapper.camino_final:
                    print(paso)
                print("🔄 Giros:")
                print(mapper.giros_final)
                break

        except KeyboardInterrupt:
            print("\n🛑 Interrumpido por el usuario.")
            break
        except Exception as e:
            print(f"⚠️ Error: {e}")

if __name__ == "__main__":
    main()
