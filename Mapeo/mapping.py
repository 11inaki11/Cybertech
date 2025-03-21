import networkx as nx
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

plt.ion()  # 🔄 Modo interactivo activado

class MicromouseMapper:
    def __init__(self, d, k):
        self.graph = nx.DiGraph()
        self.d = d
        self.k = k
        self.current_position = (0, 0) 
        self.current_distance = 0
        self.visited = set()
        self.stack = [(0, 0)]  # Para backtracking


        # 🔲 Inicializar figura de matplotlib
        self.fig, self.ax = plt.subplots(figsize=(10, 8))

        self.graph.add_node(self.current_position, distancia=self.current_distance, hijo_dcha=False, hijo_frente=False, hijo_izq=False, explorado=True)

    def update_map(self, ur_front, ur_right, ur_left, orientation):
        position = self.current_position
        distance = self.current_distance

        if position in self.visited:
            return

        can_go_front = ur_front > self.k
        can_go_right = ur_right > self.k
        can_go_left = ur_left > self.k

        self.graph.nodes[position].update({
            'distancia': distance,
            'hijo_dcha': can_go_right,
            'hijo_frente': can_go_front,
            'hijo_izq': can_go_left,
            'explorado': True
        })

        self.add_unexplored_nodes(position, can_go_front, can_go_right, can_go_left, orientation)
        self.visited.add(position)

        

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

    def elegir_direccion(self, orientation):
        node = self.graph.nodes[self.current_position]
        if node['hijo_frente']:
            return 'recto'
        elif node['hijo_dcha']:
            return 'derecha'
        elif node['hijo_izq']:
            return 'izquierda'
        else:
            return 'backtrack'

    def avanzar(self, direccion, orientation):
        if direccion == 'backtrack':
            destino = self.buscar_nodo_no_explorado()
            if destino:
                print(f"🧭 Retroceder a nodo sin explorar: {destino}")
                self.current_position = destino
                self.current_distance += self.d
            else:
                print("✅ Exploración completa.")
                exit()
        else:
            nueva_pos = self.get_new_pos(self.current_position, direccion, orientation)
            self.graph.nodes[self.current_position][f"hijo_{direccion}"] = False  # Marcar como visitado
            self.current_position = nueva_pos
            self.current_distance += self.d
            self.stack.append(nueva_pos)

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

        nx.draw(self.graph, pos, with_labels=True, node_size=800, edge_color="gray",
                font_size=8, font_weight="bold", arrows=True, ax=self.ax)
        nx.draw_networkx_nodes(self.graph, pos, nodelist=explored_nodes,
                            node_color="lightblue", node_size=800, ax=self.ax)
        nx.draw_networkx_nodes(self.graph, pos, nodelist=unexplored_nodes,
                            node_color="red", node_size=800, ax=self.ax)
        labels = {node: f"D:{data['distancia']}" if data.get('explorado', False) else "?" 
                for node, data in self.graph.nodes(data=True)}
        nx.draw_networkx_labels(self.graph, pos, labels, font_size=8, verticalalignment='bottom', ax=self.ax)

        self.ax.set_title("Mapa del Laberinto (Explorado y No Explorado)")
        self.ax.grid(True)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



# ================
# 🧪 Interacción
# ================

d = 10
k = 5
mapper = MicromouseMapper(d, k)

while True:
    print(f"\n📍 Posición actual: {mapper.current_position} - Distancia: {mapper.current_distance}")
    try:
        ur_front = float(input("Distancia sensor frontal: "))
        ur_right = float(input("Distancia sensor derecho: "))
        ur_left = float(input("Distancia sensor izquierdo: "))
        orientation = input("Orientación actual (N/S/E/O): ").strip().upper()

        mapper.update_map(ur_front, ur_right, ur_left, orientation)
        direction = mapper.elegir_direccion(orientation)
        print(f"➡️  Debes avanzar hacia: {direction.upper()}")
        mapper.avanzar(direction, orientation)

        # 🔄 Mostrar el mapa actualizado
        mapper.dibujar_grafo()

    except KeyboardInterrupt:
        print("\n⏹ Exploración interrumpida por el usuario.")
        break

# Al final puedes usar:
# mapper.mostrar_grafo()
# mapper.dibujar_grafo()
