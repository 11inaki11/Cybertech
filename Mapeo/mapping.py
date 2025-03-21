import networkx as nx
import matplotlib.pyplot as plt

class MicromouseMapper:
    def __init__(self, d, k):
        """ Inicializa el mapeador del Micromouse con visualización mejorada. """
        self.graph = nx.DiGraph()  # Grafo dirigido
        self.d = d  # Distancia entre celdas
        self.k = k  # Umbral de detección de paredes
        self.current_position = (0, 0)  # Posición inicial
        self.current_distance = 0  # Distancia recorrida
        self.visited = set()  # Conjunto de posiciones visitadas

        # Agregar nodo inicial
        self.graph.add_node(self.current_position, distancia=self.current_distance, hijo_dcha=False, hijo_frente=False, hijo_izq=False, explorado=True)

    def update_map(self, position, distance, ur_front, ur_right, ur_left, orientation):
        """ Actualiza el grafo con la nueva posición del robot y los datos de los sensores. """
        if position in self.visited:
            return

        # Determinar direcciones posibles
        can_go_front = ur_front > self.k
        can_go_right = ur_right > self.k
        can_go_left = ur_left > self.k

        # Agregar nodo explorado al grafo
        self.graph.add_node(position, distancia=distance, hijo_dcha=can_go_right, hijo_frente=can_go_front, hijo_izq=can_go_left, explorado=True)
        
        # Conectar con la posición anterior
        prev_pos = self.current_position
        if prev_pos != position:
            self.graph.add_edge(prev_pos, position, move=self.get_move(prev_pos, position))

        # Agregar nodos no explorados si hay caminos disponibles
        self.add_unexplored_nodes(position, can_go_front, can_go_right, can_go_left, orientation)

        # Actualizar estado
        self.visited.add(position)
        self.current_position = position
        self.current_distance = distance

    def add_unexplored_nodes(self, position, can_go_front, can_go_right, can_go_left, orientation):
        """ Agrega nodos no explorados en direcciones disponibles. """
        x, y = position

        if can_go_front:
            new_pos = (x, y + self.d) if orientation == 'N' else (x, y - self.d) if orientation == 'S' else (x + self.d, y) if orientation == 'E' else (x - self.d, y)
            if new_pos not in self.graph.nodes:
                self.graph.add_node(new_pos, distancia=None, explorado=False)
                self.graph.add_edge(position, new_pos, move='recto')

        if can_go_right:
            new_pos = (x + self.d, y) if orientation == 'N' else (x - self.d, y) if orientation == 'S' else (x, y - self.d) if orientation == 'E' else (x, y + self.d)
            if new_pos not in self.graph.nodes:
                self.graph.add_node(new_pos, distancia=None, explorado=False)
                self.graph.add_edge(position, new_pos, move='derecha')

        if can_go_left:
            new_pos = (x - self.d, y) if orientation == 'N' else (x + self.d, y) if orientation == 'S' else (x, y + self.d) if orientation == 'E' else (x, y - self.d)
            if new_pos not in self.graph.nodes:
                self.graph.add_node(new_pos, distancia=None, explorado=False)
                self.graph.add_edge(position, new_pos, move='izquierda')

    def get_move(self, prev_pos, new_pos):
        """ Determina la acción tomada para ir de prev_pos a new_pos. """
        dx = new_pos[0] - prev_pos[0]
        dy = new_pos[1] - prev_pos[1]

        if dx > 0:
            return 'derecha'
        elif dx < 0:
            return 'izquierda'
        elif dy > 0:
            return 'recto'
        else:
            return 'desconocido'

    def dibujar_grafo(self):
        """ Representación gráfica del grafo con nodos explorados y no explorados diferenciados. """
        plt.figure(figsize=(10, 8))
        pos = {node: node for node in self.graph.nodes()}  # Posiciones reales de los nodos

        # Separar nodos explorados y no explorados
        explored_nodes = [node for node, data in self.graph.nodes(data=True) if data.get('explorado', False)]
        unexplored_nodes = [node for node, data in self.graph.nodes(data=True) if not data.get('explorado', False)]

        # Dibujar nodos explorados en azul y no explorados en rojo
        nx.draw(self.graph, pos, with_labels=True, node_size=800, edge_color="gray", font_size=8, font_weight="bold", arrows=True)
        nx.draw_networkx_nodes(self.graph, pos, nodelist=explored_nodes, node_color="lightblue", node_size=800)
        nx.draw_networkx_nodes(self.graph, pos, nodelist=unexplored_nodes, node_color="red", node_size=800)

        # Añadir etiquetas con las propiedades de cada nodo
        labels = {node: f"D:{data['distancia']}" if data.get('explorado', False) else "?" for node, data in self.graph.nodes(data=True)}
        nx.draw_networkx_labels(self.graph, pos, labels, font_size=8, verticalalignment='bottom')

        plt.title("Mapa del Laberinto (Explorado y No Explorado)")
        plt.grid(True)
        plt.show()


    def mostrar_grafo(self):
            """ Muestra el grafo en consola. """
            for node, data in self.graph.nodes(data=True):
                print(f"Posición: {node}, Datos: {data}")
            print("\nAristas:")
            for edge in self.graph.edges(data=True):
                print(edge)


# =============================
# 🚀 Ejemplo de Uso:
# =============================

d = 10  # Separación entre casillas
k = 5   # Umbral de detección de paredes

mapper = MicromouseMapper(d, k)

# Simulación de exploración (posición, distancia, sensor_front, sensor_right, sensor_left, orientación)
mapper.update_map((0, 10), 10, 15, 3, 12, 'N')  # Puede ir recto e izquierda
mapper.update_map((0, 20), 20, 10, 12, 0, 'N')  # Puede ir recto y derecha
mapper.update_map((0, 30), 20, 10, 2, 0, 'N')  
mapper.update_map((0, 40), 2, 10, 2, 0, 'N') 
mapper.update_map((10, 40), 20, 1, 2, 0, 'E') 
mapper.update_map((20, 40), 20, 1, 2, 0, 'E') 
mapper.update_map((30, 40), 2, 10, 20, 0, 'E') 
mapper.update_map((40, 40), 2, 10, 2, 0, 'N') 
# mapper.update_map((10, 20), 30, 5, 15, 15, 'E') # Puede ir derecha e izquierda
# mapper.update_map((10, 30), 40, 12, 10, 15, 'N')  # Puede ir recto e izquierda
# mapper.update_map((10, 40), 50, 15, 5, 10, 'N')   # Puede ir recto e izquierda
# mapper.update_map((20, 40), 60, 10, 10, 10, 'E')  # Puede ir en todas direcciones
# mapper.update_map((30, 40), 70, 5, 15, 12, 'E')   # Puede ir derecha e izquierda
# mapper.update_map((30, 50), 80, 15, 10, 5, 'N')   # Puede ir recto y derecha
# mapper.update_map((40, 50), 90, 10, 12, 10, 'E')  # Puede ir en todas direcciones
# mapper.update_map((40, 60), 100, 15, 5, 10, 'N')  # Puede ir recto e izquierda
# mapper.update_map((50, 60), 110, 5, 15, 12, 'E')  # Puede ir derecha e izquierda
# mapper.update_map((50, 70), 120, 15, 10, 5, 'N')  # Puede ir recto y derecha
# mapper.update_map((60, 70), 130, 10, 12, 10, 'E') # Puede ir en todas direcciones
# mapper.update_map((70, 70), 140, 15, 5, 10, 'E')  # Puede ir derecha e izquierda
# mapper.update_map((70, 80), 150, 5, 15, 12, 'N')  # Puede ir recto e izquierda
# mapper.update_map((80, 80), 160, 15, 10, 5, 'E')  # Puede ir recto y derecha
# mapper.update_map((80, 90), 170, 10, 12, 10, 'N') # Puede ir en todas direcciones
# mapper.update_map((90, 90), 180, 15, 5, 10, 'E')  # Puede ir derecha e izquierda
# mapper.update_map((90, 100), 190, 5, 15, 12, 'N') # Puede ir recto e izquierda
# mapper.update_map((100, 100), 200, 15, 10, 5, 'E')# Puede ir recto y derecha
# mapper.update_map((100, 110), 210, 10, 12, 10, 'N')# Puede ir en todas direcciones


# Mostrar el grafo en texto y graficarlo
mapper.mostrar_grafo()
mapper.dibujar_grafo()
