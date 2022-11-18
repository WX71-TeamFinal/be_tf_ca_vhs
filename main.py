import sys
import PySimpleGUI as sg
import geopy.distance as gd
import folium
import io
from tkinter import *
from tkhtmlview import HTMLText, RenderHTML, HTMLLabel, html_parser
import PySimpleGUI as sg
import codecs
import webview
from flask import Flask
import flask_cors

Vertices = ["1","2","3","4","5","6","7","8","9","10","11"]

points = [(-12.084999, -77.04896),(-12.07581, -77.04896),(-12.08003, -77.04015),(-12.07394, -77.04322),(-12.07884, -77.05015),(-12.07395, -77.04887),(-12.07509, -77.05358),(-12.07496, -77.04666),(-12.07411, -77.04878),(-12.06567, -77.04534),(-12.08012, -77.04039)]

class Punto:
    def __init__(self, _id, name,  alt_lat):
        self._id = _id
        self.name = name
        self.alt_lat = alt_lat

arreglo_de_puntos = []

for i in range(len(points)):
    arreglo_de_puntos.append(Punto(str(i + 1),"palabra "+str(i + 1), points[i]))
 #   arreglo_de_puntos = [Punto(vertice, (-12.084999, -77.04896)), Punto("segundo", "segundo"), Punto("tercero", "tercero"), Punto("cuarto", "cuarto")]
print(arreglo_de_puntos[10].alt_lat)

graph = {
    "1": ["5", "11", "9"],
    "2": ["5", "8", "9"],
    "3": ["11", "4"],
    "4": ["3", "8", "10"],
    "5": ["1", "2", "7"],
    "6": ["9"],
    "7": ["5"],
    "8": ["2", "4"],
    "9": ["1","2", "6"],
    "10": ["4"],
    "11": ["1","3"]
}

# Minus 1
#print(arreglo_de_puntos[0].alt_lat)
#print(arreglo_de_puntos[4].alt_lat)
#print(arreglo_de_puntos[10].alt_lat)
#print(arreglo_de_puntos[8].alt_lat)
#print(gd.geodesic(arreglo_de_puntos[0].alt_lat, arreglo_de_puntos[4].alt_lat).km)
#print(gd.geodesic(arreglo_de_puntos[0].alt_lat, arreglo_de_puntos[10].alt_lat).km)
#print(gd.geodesic(arreglo_de_puntos[0].alt_lat, arreglo_de_puntos[8].alt_lat).km)


#print(type(graph))
for key, value in graph.items():
    primeraParte = 'init_graph["'
    segundaParte = '"]["'
    terceraParte = '"] = '
    for valores in value:
        print(primeraParte + key + segundaParte + valores + terceraParte + str(gd.geodesic(arreglo_de_puntos[int(key) - 1].alt_lat, arreglo_de_puntos[int(valores) - 1].alt_lat).km))



visited = [] # List for visited nodes.
queue = []     #Initialize a queue

def bfs(visited, graph, node): #function for BFS
    visited.append(node)
    queue.append(node)

    while queue:          # Creating loop to visit each node
        m = queue.pop(0)
        print (m, end = " ")

        for neighbour in graph[m]:
            if neighbour not in visited:
                visited.append(neighbour)
                queue.append(neighbour)

nuevo_points = [(-12.084999, -77.04896),(-12.07395, -77.04887),(-12.07404, -77.04148),(-12.08003, -77.04015),(-12.07496, -77.04666),(-12.07394, -77.04322),(-12.07411, -77.04878),(-12.06567, -77.04534),(-12.07884, -77.05015),(-12.07509, -77.05358),(-12.08012, -77.04039)]

class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)

    def construct_graph(self, nodes, init_graph):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = {}
        for node in nodes:
            graph[node] = {}

        graph.update(init_graph)

        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value

        return graph

    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes

    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections

    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]

def dijkstra_algorithm(graph, start_node):
    unvisited_nodes = list(graph.get_nodes())

    # We'll use this dict to save the cost of visiting each node and update it as we move along the graph
    shortest_path = {}

    # We'll use this dict to save the shortest known path to a node found so far
    previous_nodes = {}

    # We'll use max_value to initialize the "infinity" value of the unvisited nodes
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0
    shortest_path[start_node] = 0

    # The algorithm executes until we visit all nodes
    while unvisited_nodes:
        # The code block below finds the node with the lowest score
        current_min_node = None
        for node in unvisited_nodes: # Iterate over the nodes
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node

        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node

        # After visiting its neighbors, we mark the node as "visited"
        unvisited_nodes.remove(current_min_node)

    return previous_nodes, shortest_path

def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    path_coord = []
    node = target_node

    while node != start_node:
        path.append(node)
        path_coord.append(arreglo_de_puntos[int(node) - 1].alt_lat)
        node = previous_nodes[node]

    # Add the start node manually
    path.append(start_node)


    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))
    print(path_coord)
    return path_coord

graph1 = {
    "1": ["5", "11", "9"],
    "2": ["5", "8", "9"],
    "3": ["11", "4"],
    "4": ["3", "8", "10"],
    "5": ["1", "2", "7"],
    "6": ["9"],
    "7": ["5"],
    "8": ["2", "4"],
    "9": ["1","2", "6"],
    "10": ["4"],
    "11": ["1","3"]
}

init_graph = {}
for node in Vertices:
    init_graph[node] = {}

for key, value in graph1.items():
    primeraParte = 'init_graph["'
    segundaParte = '"]["'
    terceraParte = '"] = '
    for valores in value:
        exec(primeraParte + key + segundaParte + valores + terceraParte + str(gd.geodesic(arreglo_de_puntos[int(key) - 1].alt_lat, arreglo_de_puntos[int(valores) - 1].alt_lat).km))

graph = Graph(Vertices, init_graph)
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node="1")
arreglo_final = print_result(previous_nodes, shortest_path, start_node="1", target_node="10")

boulder_coords = [-12.075, -77.045]

#Create the map
my_map = folium.Map(location = boulder_coords, zoom_start = 15)

#Display the map
folium.PolyLine(arreglo_final).add_to(my_map)

#add markers
i = 1
for each in arreglo_final:
    folium.Marker(each, popup= str(i)).add_to(my_map)
    i = i + 1


#
#
#
app = Flask(__name__)
cors = CORS(app, resources={r"/api/*": {"origins": "*"}})

@app.route('/api/generated_map')
@cross_origin(origin='*')
def index():
    start_coords = (46.9540700, 142.7360300)
    folium_map = folium.Map(location=start_coords, zoom_start=14)
    return my_map._repr_html_()


if __name__ == '__main__':
    app.run(debug=True)