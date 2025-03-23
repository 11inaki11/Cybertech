import networkx as nx
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import math

plt.ion()  # 🔄 Modo interactivo activado

class MicromouseMapper:
    def __init__(self, d, k, num_casillas_total):
        self.graph = nx.DiGraph()
        self.d = d # Tamaño de cada casilla
        self.k = k # Distancia mínima para considerar que hay una pared
        self.current_position = (self.d / 2, self.d / 2)
        self.current_distance = 0
        self.visited = set()
        self.stack = [self.current_position]  
        self.num_casillas_total = num_casillas_total
        self.lado = int(num_casillas_total ** 0.5)  # Suponemos mapa cuadrado
        self.centro = (
        (self.lado // 2) * self.d - self.d / 2,
        (self.lado // 2) * self.d - self.d / 2
        )
        self.walls = []
        # Coordenadas de las 4 casillas centrales (centros)
        offset = self.d / 2
        x1 = (self.lado // 2 - 1) * self.d + offset
        x2 = (self.lado // 2) * self.d + offset
        y1 = (self.lado // 2 - 1) * self.d + offset
        y2 = (self.lado // 2) * self.d + offset
        self.celdas_centro = [(x1, y1), (x1, y2), (x2, y1), (x2, y2)]



        # 🔲 Inicializar figura de matplotlib
        self.fig, self.ax = plt.subplots(figsize=(10, 8))

        self.graph.add_node(self.current_position, distancia=self.current_distance, hijo_dcha=False, hijo_frente=False, hijo_izq=False, explorado=True)

        # 🧱 Añadir pared por debajo del nodo inicial (sur)
        pared_inicial = self.calcular_borde_pared(self.current_position, 'recto', 'S')
        self.walls.append(pared_inicial)


    def update_map(self, ur_front, ur_right, ur_left, orientation):
        position = self.current_position
        distance = self.current_distance

        if position in self.visited:
            return

        can_go_front = ur_front > self.k
        can_go_right = ur_right > self.k
        can_go_left = ur_left > self.k

        # Almacenar paredes detectadas
        if not can_go_front:
            wall = self.calcular_borde_pared(self.current_position, 'recto', orientation)
            self.walls.append(wall)
        if not can_go_right:
            wall = self.calcular_borde_pared(self.current_position, 'derecha', orientation)
            self.walls.append(wall)

        if not can_go_left:
            wall = self.calcular_borde_pared(self.current_position, 'izquierda', orientation)
            self.walls.append(wall)

        self.graph.nodes[position].update({
            'distancia': distance,
            'hijo_dcha': can_go_right,
            'hijo_frente': can_go_front,
            'hijo_izq': can_go_left,
            'explorado': True
        })

        self.add_unexplored_nodes(position, can_go_front, can_go_right, can_go_left, orientation)
        self.visited.add(position)

    def calcular_borde_pared(self, posicion, direccion, orientacion):
        x, y = posicion
        mitad = self.d / 2
        if orientacion == 'N':
            if direccion == 'recto':
                return [(x - mitad, y + mitad), (x + mitad, y + mitad)]
            elif direccion == 'derecha':
                return [(x + mitad, y - mitad), (x + mitad, y + mitad)]
            elif direccion == 'izquierda':
                return [(x - mitad, y - mitad), (x - mitad, y + mitad)]
        elif orientacion == 'S':
            if direccion == 'recto':
                return [(x + mitad, y - mitad), (x - mitad, y - mitad)]
            elif direccion == 'derecha':
                return [(x - mitad, y + mitad), (x - mitad, y - mitad)]
            elif direccion == 'izquierda':
                return [(x + mitad, y + mitad), (x + mitad, y - mitad)]
        elif orientacion == 'E':
            if direccion == 'recto':
                return [(x + mitad, y + mitad), (x + mitad, y - mitad)]
            elif direccion == 'derecha':
                return [(x - mitad, y - mitad), (x + mitad, y - mitad)]
            elif direccion == 'izquierda':
                return [(x - mitad, y + mitad), (x + mitad, y + mitad)]
        elif orientacion == 'O':
            if direccion == 'recto':
                return [(x - mitad, y - mitad), (x - mitad, y + mitad)]
            elif direccion == 'derecha':
                return [(x + mitad, y + mitad), (x + mitad, y - mitad)]
            elif direccion == 'izquierda':
                return [(x - mitad, y + mitad), (x - mitad, y - mitad)]

    def add_unexplored_nodes(self, position, can_go_front, can_go_right, can_go_left, orientation):
        x, y = position
        directions = []

        if can_go_front:
            new_pos = self.get_new_pos(position, 'recto', orientation)
            directions.append(('recto', new_pos))
        if can_go_right:
            new_pos = self.get_new_pos(position, 'derecha', orientation)
            directions.append(('derecha', new_pos))
        if can_go_left:
            new_pos = self.get_new_pos(position, 'izquierda', orientation)
            directions.append(('izquierda', new_pos))

        for move, new_pos in directions:
            if new_pos not in self.graph.nodes:
                self.graph.add_node(new_pos, distancia=None, explorado=False)
                self.graph.add_edge(position, new_pos, move=move)

    def get_new_pos(self, position, direction, orientation):
        x, y = position
        if orientation == 'N':
            if direction == 'recto': return (x, y + self.d)
            if direction == 'derecha': return (x + self.d, y)
            if direction == 'izquierda': return (x - self.d, y)
        elif orientation == 'S':
            if direction == 'recto': return (x, y - self.d)
            if direction == 'derecha': return (x - self.d, y)
            if direction == 'izquierda': return (x + self.d, y)
        elif orientation == 'E':
            if direction == 'recto': return (x + self.d, y)
            if direction == 'derecha': return (x, y - self.d)
            if direction == 'izquierda': return (x, y + self.d)
        elif orientation == 'O':
            if direction == 'recto': return (x - self.d, y)
            if direction == 'derecha': return (x, y + self.d)
            if direction == 'izquierda': return (x, y - self.d)

    def calcular_angulo_hacia_centro(self, origen):
        dx = self.centro[0] - origen[0]
        dy = self.centro[1] - origen[1]
        angulo_rad = math.atan2(dx, dy)  # ⚠️ dx primero para que 0º sea norte
        angulo_deg = (math.degrees(angulo_rad)) % 360
        return angulo_deg


    def elegir_direccion(self, orientation):
        node = self.graph.nodes[self.current_position]
        posibles_direcciones = []

        for nombre, clave in [('frente', 'recto'), ('dcha', 'derecha'), ('izq', 'izquierda')]:
            if node.get(f'hijo_{nombre}', False):
                destino = self.get_new_pos(self.current_position, clave, orientation)
                if destino not in self.graph.nodes or not self.graph.nodes[destino].get('explorado', False):
                    posibles_direcciones.append((clave, nombre, destino))

        if not posibles_direcciones:
            return 'backtrack'

        # 🧭 Ángulo hacia el centro desde la posición actual
        angulo_objetivo = self.calcular_angulo_hacia_centro(self.current_position)

        def angulo_direccion(destino):
            dx = destino[0] - self.current_position[0]
            dy = destino[1] - self.current_position[1]
            ang = math.atan2(dx, dy)
            return (math.degrees(ang)) % 360


        # 🎯 Elegir la opción cuyo ángulo esté más cerca del objetivo
        mejores_opciones = sorted(
            posibles_direcciones,
            key=lambda x: (self.distancia_angular(angulo_direccion(x[2]), angulo_objetivo),  # distancia angular al centro
                        ['frente', 'dcha', 'izq'].index(x[1]))  # desempate por prioridad
        )

        return mejores_opciones[0][0]

    def distancia_angular(self, a, b):
        diff = abs(a - b) % 360
        return min(diff, 360 - diff)


    def avanzar(self, direccion, orientation):
        if direccion == 'backtrack':
            destino = self.buscar_nodo_no_explorado()
            if destino:
                print(f"🧭 Retroceder a nodo sin explorar: {destino}")
                self.current_position = destino
                # Recuperar la distancia ya registrada del nodo destino
                self.current_distance = self.graph.nodes[destino]['distancia']
            else:
                print("✅ Exploración completa.")
                exit()
        else:
            nueva_pos = self.get_new_pos(self.current_position, direccion, orientation)
            self.graph.nodes[self.current_position][f"hijo_{direccion}"] = False  # Marcar como visitado
            self.current_position = nueva_pos
            self.stack.append(nueva_pos)

            # Calcular la distancia desde el nodo anterior
            distancia_padre = self.graph.nodes[self.stack[-2]]['distancia']
            self.graph.nodes[nueva_pos]['distancia'] = distancia_padre + self.d
            self.current_distance = self.graph.nodes[nueva_pos]['distancia']
        
        # Verificar si hemos llegado a una de las casillas centrales
        if self.current_position in self.celdas_centro:
            print("🎯 ¡Centro del laberinto alcanzado!")
            camino = self.stack.copy()
            movimientos, giros = self.obtener_camino_y_giros(camino)
            print("\n📌 Secuencia de coordenadas:")
            for p in camino:
                print(p)
            print("\n🔁 Secuencia de movimientos:")
            print(movimientos)
            print("\n🔄 Secuencia de giros:")
            print(giros)
            exit()

    def obtener_camino_y_giros(self, camino):
        movimientos = []
        giros = []
        orientacion_actual = 'N'

        for i in range(1, len(camino)):
            x1, y1 = camino[i-1]
            x2, y2 = camino[i]

            dx = x2 - x1
            dy = y2 - y1

            # Detectar dirección del movimiento
            if dx == self.d:
                direccion = 'E'
            elif dx == -self.d:
                direccion = 'O'
            elif dy == self.d:
                direccion = 'N'
            elif dy == -self.d:
                direccion = 'S'
            else:
                direccion = '?'

            movimientos.append((x2, y2))

            # Calcular giro necesario
            giro = self.calcular_giro(orientacion_actual, direccion)
            giros.append(giro)
            orientacion_actual = direccion

        return movimientos, giros
    
    def calcular_giro(self, desde, hacia):
        orden = ['N', 'E', 'S', 'O']
        i_desde = orden.index(desde)
        i_hacia = orden.index(hacia)
        diff = (i_hacia - i_desde) % 4
        if diff == 0:
            return 'seguir recto'
        elif diff == 1:
            return 'girar derecha'
        elif diff == 2:
            return 'giro en U'
        elif diff == 3:
            return 'girar izquierda'



    def buscar_nodo_no_explorado(self):
        for node in reversed(self.stack):
            datos = self.graph.nodes[node]
            orientation_options = ['N', 'S', 'E', 'O']

            for orientacion in orientation_options:
                if datos.get('hijo_frente'):
                    pos_hijo = self.get_new_pos(node, 'recto', orientacion)
                    if pos_hijo in self.graph.nodes and not self.graph.nodes[pos_hijo].get('explorado', False):
                        return node
                if datos.get('hijo_dcha'):
                    pos_hijo = self.get_new_pos(node, 'derecha', orientacion)
                    if pos_hijo in self.graph.nodes and not self.graph.nodes[pos_hijo].get('explorado', False):
                        return node
                if datos.get('hijo_izq'):
                    pos_hijo = self.get_new_pos(node, 'izquierda', orientacion)
                    if pos_hijo in self.graph.nodes and not self.graph.nodes[pos_hijo].get('explorado', False):
                        return node
        return None


    def mostrar_grafo(self):
        for node, data in self.graph.nodes(data=True):
            print(f"Posición: {node}, Datos: {data}")
        print("\nAristas:")
        for edge in self.graph.edges(data=True):
            print(edge)

    def dibujar_grafo(self):
        self.ax.clear()
        pos = {node: node for node in self.graph.nodes()}
        explored_nodes = [node for node, data in self.graph.nodes(data=True) if data.get('explorado', False)]
        unexplored_nodes = [node for node, data in self.graph.nodes(data=True) if not data.get('explorado', False)]

        # Dibujar fondo de cuadrícula
        for x in range(0, self.lado * self.d + 1, self.d):
            self.ax.axvline(x=x, color='lightgray', linestyle='--', linewidth=0.5)
        for y in range(0, self.lado * self.d + 1, self.d):
            self.ax.axhline(y=y, color='lightgray', linestyle='--', linewidth=0.5)


        nx.draw(self.graph, pos, with_labels=True, node_size=800, edge_color="gray",
                font_size=8, font_weight="bold", arrows=True, ax=self.ax)
        nx.draw_networkx_nodes(self.graph, pos, nodelist=explored_nodes,
                            node_color="lightblue", node_size=800, ax=self.ax)
        nx.draw_networkx_nodes(self.graph, pos, nodelist=unexplored_nodes,
                            node_color="red", node_size=800, ax=self.ax)
        labels = {node: f"D:{data['distancia']}" if data.get('explorado', False) else "?" 
                for node, data in self.graph.nodes(data=True)}
        nx.draw_networkx_labels(self.graph, pos, labels, font_size=8, verticalalignment='bottom', ax=self.ax)

        # 🔶 Dibujar el nodo actual en dorado
        nx.draw_networkx_nodes(self.graph, pos,
            nodelist=[self.current_position],
            node_color="gold", node_size=900, ax=self.ax
        )

        # 🏷 Etiquetas con la distancia
        labels = {
            node: f"D:{data['distancia']}" if data.get('explorado', False) else "?" 
            for node, data in self.graph.nodes(data=True)
        }
        nx.draw_networkx_labels(self.graph, pos, labels, font_size=8, verticalalignment='bottom', ax=self.ax)


        self.ax.set_title("Mapa del Laberinto (Explorado y No Explorado)")
        self.ax.set_xlim(-self.d, self.lado * self.d)
        self.ax.set_ylim(-self.d, self.lado * self.d)
        self.ax.set_aspect('equal')

        self.ax.grid(True)
        self.fig.canvas.draw()
        # Dibujar paredes como líneas negras
        for p1, p2 in self.walls:
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='black', linewidth=2)


        self.fig.canvas.flush_events()
        # Dibujar el centro del laberinto como punto verde
        self.ax.plot(self.centro[0], self.centro[1], 'go', markersize=12)
        # Flecha al centro
        self.ax.arrow(
            self.current_position[0], self.current_position[1],
            self.centro[0] - self.current_position[0],
            self.centro[1] - self.current_position[1],
            head_width=1, head_length=1, fc='green', ec='green', linewidth=1.5, alpha=0.6
        )



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



# ================
# 🧪 Interacción
# ================

d = 18
k = 9
num_casillas_total = 64  # por ejemplo, un laberinto de 16x16
mapper = MicromouseMapper(d, k, num_casillas_total)

while True:
    print(f"\n📍 Posición actual: {mapper.current_position} - Distancia desde el inicio: {mapper.graph.nodes[mapper.current_position]['distancia']}")
    try:
        ur_front = float(input("Distancia sensor frontal: "))
        ur_right = float(input("Distancia sensor derecho: "))
        ur_left = float(input("Distancia sensor izquierdo: "))
        angulo = float(input("Ángulo de orientación (grados): "))
        orientation = grados_a_orientacion(angulo)
        print(f"📐 Orientación aproximada: {orientation}")


        mapper.update_map(ur_front, ur_right, ur_left, orientation)
        direction = mapper.elegir_direccion(orientation)
        print(f"➡️  Debes avanzar hacia: {direction.upper()}")
        mapper.avanzar(direction, orientation)

        # 🔄 Mostrar el mapa actualizado
        mapper.dibujar_grafo()

    except KeyboardInterrupt:
        print("\n⏹ Exploración interrumpida por el usuario.")
        break
