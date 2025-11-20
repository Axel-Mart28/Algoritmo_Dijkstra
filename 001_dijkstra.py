import networkx as nx           # Libreria para crear y manipular la estructura del grafo (nodos y aristas)
import matplotlib.pyplot as plt # Libreria para dibujar el grafo en una ventana
import time                     # Libreria para controlar el tiempo 

#La clase Vertice representa cada nodo del grafo.
class Vertice:
    def __init__(self, i):
        self.id = i                 # Identificador del vertice (ej: 'A', 'B')
        self.vecinos = []           # Lista de tuplas (vecino, peso) a los que esta conectado
        self.visitado = False       # Bandera para saber si ya procesamos este nodo definitivamente
        self.padre = None           # Guarda de que nodo venimos (clave para reconstruir el camino al final)
        self.distancia = float('inf') # Distancia inicial es Infinito (desconocida)

    # Método para conectar este vertice con otro
    def agregar_vecino(self, v, p):
        # Si el vecino no esta ya en la lista, lo agregamos junto con el peso de la arista
        if v not in self.vecinos:
            self.vecinos.append((v, p))


# La clase Grafo controla la lógica del algoritmo y almacena todos los vertices.
class Grafo:
    def __init__(self):
        self.vertices = {} # Diccionario para acceso rápido: {'A': ObjetoVerticeA, 'B': ObjetoVerticeB...}

    # Crea un nuevo nodo y lo guarda en el diccionario
    def agregar_vertice(self, id):
        if id not in self.vertices:
            self.vertices[id] = Vertice(id)

    # Crea una conexión entre dos nodos (Grafo No Dirigido: ida y vuelta)
    def agregar_arista(self, a, b, p):
        if a in self.vertices and b in self.vertices:
            self.vertices[a].agregar_vecino(b, p) # Conecta A con B
            self.vertices[b].agregar_vecino(a, p) # Conecta B con A

    # Metodo auxiliar para encontrar el nodo no visitado con la menor distancia acumulada
    def minimo(self, lista):
        if len(lista) > 0:
            # Suponemos que el primero es el menor
            m = self.vertices[lista[0]].distancia
            v = lista[0]
            # Recorremos la lista buscando si hay uno menor
            for e in lista:
                if m > self.vertices[e].distancia:
                    m = self.vertices[e].distancia
                    v = e
            return v # Retorna el ID del vértice ganador
        return None

    # Metodo para reconstruir el camino final una vez ejecutado Dijkstra
    def camino(self, a, b):
        camino = []
        actual = b
        # Vamos hacia atras: desde el destino (b) buscando a sus padres hasta llegar al inicio
        while actual is not None:
            camino.append(actual)
            actual = self.vertices[actual].padre
        camino.reverse() # Invertimos la lista para que vaya de Inicio -> Fin
        return [camino, self.vertices[b].distancia]

    # Metodo auxiliar para ver el estado de todos los nodos en texto
    def imprimir_grafo(self):
        for v in self.vertices:
            print(f"Vértice {v}: Distancia {self.vertices[v].distancia}, Padre: {self.vertices[v].padre}")

    # implementacion del algoritmo de Dijkstra
    def dijkstra(self, a):
        print(f"\n--- INICIANDO SIMULACIÓN DIJKSTRA DESDE '{a}' ---")
        
        if a in self.vertices:
            # 1. Inicializacion
            self.vertices[a].distancia = 0 # La distancia al nodo inicial siempre es 0
            actual = a
            noVisitados = [] # Lista de nodos pendientes por procesar

            # Preparamos todos los vértices
            for v in self.vertices:
                if v != a:
                    self.vertices[v].distancia = float('inf') # Los demás inician en infinito
                self.vertices[v].padre = None
                noVisitados.append(v) # Agregamos todos a la lista de pendientes

            # 2. Bucle principal: Mientras queden nodos por visitar
            while len(noVisitados) > 0:
                # Imprimimos estado actual 
                print(f"\n--> Visitando nodo actual: [{actual}]")
                print(f"    Nodos pendientes por visitar: {noVisitados}")
                time.sleep(1) # Pausa de 1 segundo para que el usuario pueda leer

                # 3. Revisamos a todos los vecinos del nodo actual
                for vecino in self.vertices[actual].vecinos:
                    if self.vertices[vecino[0]].visitado == False:
                        peso = vecino[1]        # Cuanto cuesta ir al vecino
                        nodo_vecino = vecino[0] # El ID del vecino (ej. 'B')
                        
                        print(f"    Analizando vecino '{nodo_vecino}' con peso de arista {peso}...", end=" ")
                        
                        # 4. RELAJACIÓN (Relaxation): El corazón de Dijkstra
                        # Si la distancia hasta 'actual' + el peso del enlace es MENOR que la distancia que ya tenía el vecino, actualizamos.
                        if self.vertices[actual].distancia + peso < self.vertices[vecino[0]].distancia:
                            self.vertices[nodo_vecino].distancia = self.vertices[actual].distancia + peso
                            self.vertices[nodo_vecino].padre = actual # Guardamos que venimos de 'actual'
                            print(f"Actualizado, Nuevo camino óptimo encontrado. Distancia acumulada: {self.vertices[nodo_vecino].distancia}")
                        else:
                            print("No se actualiza (El camino que ya conocíamos era mejor o igual).")
                        time.sleep(0.5) # Pequeña pausa para ver el analisis de cada vecino

                # 5. Marcamos el nodo actual como visitado para no volver a el
                self.vertices[actual].visitado = True
                noVisitados.remove(actual)
                
                # 6. Seleccionamos el siguiente nodo a visitar (el que tenga menor distancia)
                actual = self.minimo(noVisitados)
                
                # Si quedan nodos pero tienen distancia infinita, son inalcanzables, terminamos.
                if actual is None and len(noVisitados) > 0:
                    break

# FUNCION PARA GRAFICAR 
def graficar_resultado(grafo_obj, camino_optimo):
    # Creamos un objeto grafo de la librería NetworkX
    G = nx.Graph()
    
    # Copiamos los datos de nuestra clase Grafo a NetworkX
    for v_id, v_obj in grafo_obj.vertices.items():
        G.add_node(v_id)
        for vecino, peso in v_obj.vecinos:
            G.add_edge(v_id, vecino, weight=peso)

    # Calculamos la posición visual de los nodos (layout)
    pos = nx.spring_layout(G) 
    
    plt.figure(figsize=(8, 6)) # Tamaño de la ventana
    
    # 1. Dibujamos el grafo base (nodos azules, líneas negras)
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=15)
    
    # 2. Dibujamos las etiquetas de los pesos en las líneas
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    # 3. RESALTAMOS EL CAMINO GANADOR
    # Creamos pares de aristas para dibujar el camino (ej: A->B, B->C)
    path_edges = list(zip(camino_optimo, camino_optimo[1:]))
    
    # Pintamos los nodos del camino en verde
    nx.draw_networkx_nodes(G, pos, nodelist=camino_optimo, node_color='lightgreen', node_size=2000)
    # Pintamos las aristas del camino en rojo y más gruesas
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=3)

    plt.title("Simulación Gráfica - Algoritmo de Dijkstra")
    plt.show() # Mostrar ventana

# --- BLOQUE PRINCIPAL DE EJECUCION ---
if __name__ == "__main__":
    # Instanciamos el grafo
    g = Grafo()
    
    # Agregamos vértices (Nodos)
    g.agregar_vertice('A')
    g.agregar_vertice('B')
    g.agregar_vertice('C')
    g.agregar_vertice('D')
    g.agregar_vertice('E')

    # Agregamos aristas (Conexiones y Pesos)
    g.agregar_arista('A', 'B', 1)
    g.agregar_arista('A', 'C', 4)
    g.agregar_arista('B', 'C', 2)
    g.agregar_arista('B', 'D', 5)
    g.agregar_arista('C', 'D', 1)
    g.agregar_arista('D', 'E', 3)

    # Ejecutamos la simulación en consola
    g.dijkstra('A')
    
    # Obtenemos el resultado final
    camino, distancia = g.camino('A', 'E')
    print(f"\n--- RESULTADO FINAL ---")
    print(f"El camino más corto de 'A' a 'E' es: {camino}")
    print(f"Costo total (Distancia): {distancia}")
    
    # Mostramos el grafico
    print("\nAbríendo ventana gráfica...")
    graficar_resultado(g, camino)