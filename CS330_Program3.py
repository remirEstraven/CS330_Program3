# Programming Assignment 3
# Class: CS 330-01
# Term: Fall 2025
# Author: Deanna Deylami & Lucas Geiger
# Date: 11/16/2025
# Purpose: Implement & Test the A* algorithm

# Create data structure to store nodes and connections
class Graph:
    def __init__(self):
        self.graph = {}
        self.nodes = []
        self.connections = []

    def assemble_graph(self):
        for i in range(1, len(self.nodes) + 1):
            self.graph[i] = []
        
        for connection in self.connections:
            self.graph[connection.from_node].append((connection.to_node, connection.cost))

    # Returns a list of connection NodeRecords from a given NodeRecord.
    # If we need to just return the numbers of the nodes then I can change it to that
    def get_connections(self, node):
        connection_nodes = []
        connections = self.graph[node.number]
        for connection in connections:
            for node in self.nodes:
                if node.number == connection[0]:
                    connection_nodes.append(node)
        return connection_nodes

# Don't know if we need this or if we can just add some functions to Graph
class PathfindingList:
    pass

class ConnectionRecord:
    def __init__(self):
        self.number = 0
        self.from_node = 0
        self.to_node = 0
        self.cost = 0
        self.cost_plot_pos = 0
        self.type = 0

class NodeRecord:
    def __init__(self):
        self.number = 0
        self.status = 0
        self.cost_so_far = 0
        self.estimated_heuristic = 0
        self.estimated_total = 0
        self.previous_node = 0
        self.x = 0.0
        self.y = 0.0
        self.number_plot_pos = 0
        self.name_plot_pos = 0
        self.name = ""

# A* to find best path from start to goal
## Can either do it the way on the pseudocode or in the R code

graph = Graph()

# Read in nodes and put into array of NodeRecords
with open("CS 330, Pathfinding, Graph AB Nodes v3.txt") as file:
    for line in file:
        if "#" not in line:
            node_info = line.split(",")
            node = NodeRecord()
            node.number = int(node_info[1].strip())
            node.x = float(node_info[7].strip())
            node.y = float(node_info[8].strip())
            node.number_plot_pos = int(node_info[9].strip())
            node.name_plot_pos = int(node_info[10].strip())
            if node_info[11].strip() != '""':
                node.name = node_info[11].strip()
            graph.nodes.append(node)

# Read in connections and put into array of ConnectionRecords
with open("CS 330, Pathfinding, Graph AB Connections v3.txt") as file:
    for line in file:
        if "#" not in line:
            connection_info = line.split(",")
            connection = ConnectionRecord()
            connection.number = int(connection_info[1].strip())
            connection.from_node = int(connection_info[2].strip())
            connection.to_node = int(connection_info[3].strip())
            connection.cost = int(connection_info[4].strip())
            connection.cost_plot_pos = int(connection_info[5].strip())
            connection.type = int(connection_info[6].strip())
            graph.connections.append(connection)

# Assemble the full graph
graph.assemble_graph()

# Paths to find
paths = [(1, 29), (1, 38), (11, 1), (33, 66), (58, 43)]

# Print out to txt file
open("output.txt", "w").close()
with open("output.txt", "a") as file:
    file.write("Nodes\n")
    for node in graph.nodes:
        file.write(f"N {node.number} {node.status} {node.cost_so_far} {node.estimated_heuristic} {node.estimated_total} {node.previous_node} {node.x} {node.y} {node.number_plot_pos} {node.name_plot_pos} {node.name}\n")
    file.write("\nConnections\n")
    for connection in graph.connections:
        file.write(f"C {connection.number} {connection.from_node} {connection.to_node} {connection.cost} {connection.cost_plot_pos} {connection.type}\n")
    file.write("\n")
    for path in paths:
        file.write(f"Path from {path[0]} to {path[1]} = \n") # Put all that stuff here