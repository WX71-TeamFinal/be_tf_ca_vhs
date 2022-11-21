import sys
import geopy.distance as gd
import folium
import webbrowser

Vertices = ["1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17","18","19","20","21","22","23","24","25","26","27","28","29","30","31","32","33","34","35","36","37","38","39","40","41","42","43","44","45","46","47","48","49","50","51","52","53","54","55","56","57","58","59","60","61","62","63","64","65","66","67","68","69","70","71","72","73","74","75","76","77","78","79","80","81","82","83"]

points=[(-11.98076, -77.03999),(-11.98056, -77.03938),(-11.97826, -77.05301),(-11.99445, -77.0558),(-11.97767, -77.05052),(-11.99046, -77.06281),(-11.99285, -77.05153),(-11.99784, -77.05494),(-12.00324, -77.05436),(-11.9764, -77.05289),(-11.98904, -77.05889),(-11.98945, -77.05849),(-12.00606, -77.05271),(-11.97745, -77.05015),(-11.9978, -77.05415),(-11.99385, -77.05416),(-11.97417, -77.04813),(-11.99777, -77.05303),(-11.97738, -77.06335),(-11.98331, -77.06087),(-11.96705, -77.03664),(-12.00586, -77.05246),(-11.97845, -77.05348),(-11.99826, -77.0549),(-11.99408, -77.05605),(-11.99469, -77.06216),(-12.00667, -77.05267),(-11.99778, -77.05452),(-11.99384, -77.05626),(-12.00519, -77.05551),(-11.98589, -77.05692),(-11.97462, -77.04752),(-11.97668, -77.04734),(-11.99142, -77.04843),(-12.00565, -77.05263),(-11.99838, -77.05493),(-11.99577, -77.05514),(-11.99777, -77.05289),(-11.99835, -77.05021),(-12.00089, -77.05469),(-12.00127, -77.04768),(-11.99301, -77.05195),(-12.00656, -77.05224),(-12.00535, -77.05267),(-12.00131, -77.05405),(-11.98287, -77.05896),(-11.97737, -77.06362),(-11.9937, -77.06178),(-11.99419, -77.06012),(-12.00577, -77.05267),(-12.00706, -77.05359),(-11.97796, -77.04683),(-11.97325, -77.04466),(-11.97325, -77.04466),(-11.97705, -77.05064),(-12.00682, -77.05351),(-12.00641, -77.0526),(-11.99779, -77.05283),(-11.98897, -77.06271),(-11.99896, -77.04988),(-11.97941, -77.04175),(-11.97972, -77.05814),(-11.9796, -77.0463),(-12.01077, -77.05229),(-12.00132, -77.05457),(-12.0077, -77.05349),(-11.99092, -77.04717),(-11.97713, -77.04889),(-11.96705, -77.03664),(-12.01046, -77.0506),(-11.97273, -77.04501),(-11.99783, -77.05221),(-11.9777, -77.05083),(-11.97741, -77.04718),(-11.98059, -77.0595),(-11.97118, -77.04152),(-11.98152, -77.04333),(-12.0012, -77.05632),(-11.97951, -77.05771),(-11.9889, -77.05812),(-11.97866, -77.04908),(-12.00398, -77.05422),(-11.9981, -77.05063)]



class Punto:
    def __init__(self, _id, name, alt_lat):
        self._id = _id
        self.name = name
        self.alt_lat = alt_lat


arreglo_de_puntos = []

for i in range(len(points)):
    arreglo_de_puntos.append(Punto(str(i + 1), "palabra " + str(i + 1), points[i]))
#   arreglo_de_puntos = [Punto(vertice, (-12.084999, -77.04896)), Punto("segundo", "segundo"), Punto("tercero", "tercero"), Punto("cuarto", "cuarto")]
#print(arreglo_de_puntos[10].alt_lat)

graph = {
 "1": ["2","77","61","63"],
"2": ["1"],
"3": ["73","23"],
"4": ["25","37"],
"5": ["73","14","55"],
"6": ["59","12","48"],
"7": ["42","34"],
"8": ["54","28","24"],
"9": ["82","65"],
"10": ["23"],
"11": ["59","80"],
"12": ["80"],
"13": ["57","50","44"],
"14": ["5","68"],
"15": ["18","28"],
"16": ["25","42"],
"17": ["53","32"],
"18": ["15","38"],
"19": ["75","47"],
"20": ["46"],
"21": ["76"],
"22": ["50"],
"23": ["10","79","3"],
"24": ["36","8"],
"25": ["49","29","80","4","16"],
"26": ["49","48"],
"27": ["43","56","57"],
"28": ["15","8"],
"29": ["80","25"],
"30": ["50"],
"31": ["20","80","62","77"],
"32": ["17","33"],
"33": ["32","74","68"],
"34": ["7","67"],
"35": ["50"],
"36": ["24","40"],
"37": ["54","4"],
"38": ["18","58"],
"39": ["60"],
"40": ["36","65"],
"41": ["39","60","45"],
"42": ["7","16"],
"43": ["27"],
"44": ["13"],
"45": ["41","65"],
"46": ["20","75"],
"47": ["19"],
"48": ["6","49"],
"49": ["78","48","25"],
"50": ["30","13","35","22"],
"51": ["66","82","56"],
"52": ["74","63","81"],
"53": ["71","76","17"],
"54": ["8","37"],
"55": ["5"],
"56": ["51","82","27"],
"57": ["13","27"],
"58": ["72","38"],
"59": ["6","11"],
"60": ["39","41","83"],
"61": ["77","63","1"],
"62": ["75","31","79"],
"63": ["52","77","1","61"],
"64": ["70","66"],
"65": ["78","9","45","40"],
"66": ["51","64"],
"67": ["34"],
"68": ["14","33"],
"69": ["76"],
"70": ["64"],
"71": ["53"],
"72": ["83","58"],
"73": ["3","5"],
"74": ["33","52"],
"75": ["62","46","19"],
"76": ["69","53","21"],
"77": ["1","63","31"],
"78": ["65","49"],
"79": ["23","62"],
"80": ["12","11","31","29"],
"81": ["52"],
"82": ["9","56","51"],
"83": ["60","72"]
}

# Minus 1
# print(arreglo_de_puntos[0].alt_lat)
# print(arreglo_de_puntos[4].alt_lat)
# print(arreglo_de_puntos[10].alt_lat)
# print(arreglo_de_puntos[8].alt_lat)
# print(gd.geodesic(arreglo_de_puntos[0].alt_lat, arreglo_de_puntos[4].alt_lat).km)
# print(gd.geodesic(arreglo_de_puntos[0].alt_lat, arreglo_de_puntos[10].alt_lat).km)
# print(gd.geodesic(arreglo_de_puntos[0].alt_lat, arreglo_de_puntos[8].alt_lat).km)

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
        for node in unvisited_nodes:  # Iterate over the nodes
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

    #print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    #print(" -> ".join(reversed(path)))
    #print(path_coord)
    return path_coord


init_graph = {}
for node in Vertices:
    init_graph[node] = {}

for key, value in graph.items():
    primeraParte = 'init_graph["'
    segundaParte = '"]["'
    terceraParte = '"] = '
    for valores in value:
        exec(primeraParte + key + segundaParte + valores + terceraParte + str(gd.geodesic(arreglo_de_puntos[int(key) - 1].alt_lat, arreglo_de_puntos[int(valores) - 1].alt_lat).km))

graph = Graph(Vertices, init_graph)
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node="9")
print(previous_nodes)
print(shortest_path)
arreglo_final = print_result(previous_nodes, shortest_path, start_node="9", target_node="10")

boulder_coords = [-12.075, -77.045]

# Create the map
my_map = folium.Map(location=boulder_coords, zoom_start=12)

# Display the map
folium.PolyLine(arreglo_final).add_to(my_map)


# add markers
i = 1
for each in arreglo_final:
    folium.Marker(each, popup=str(i)).add_to(my_map)
    i = i + 1
my_map.save('elmapita.html')
# open html file
webbrowser.open('elmapita.html')
