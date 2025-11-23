# Programming Assignment 3
# Class: CS 330-01
# Term: Fall 2025
# Author: Deanna Deylami & Lucas Geiger
# Date: 11/16/2025
# Purpose: Implement & Test the A* algorithm

import math

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

    # Returns a list of connection NodeRecords & cost from a given NodeRecord.
    def get_connections(self, node):
        connection_nodes = []
        connections = self.graph[node.number]
        for (node_num, cost) in connections:
            for node in self.nodes:
                if node.number == node_num:
                    connection_nodes.append((node, cost))
        return connection_nodes

# Calculate heuristic value using standard Euclidean 2D distance.
def cal_heuristic(node_1, node_2):
    distance1 = (node_1.x - node_2.x)**2
    distance2 = (node_1.y - node_2.y)**2
    return math.sqrt(distance1 + distance2)

# Find & return the node with the smallest total cost.
def find_lowest(nodes):
    lowest = nodes[0]
    for node in nodes:
        if node.estimated_total < lowest.estimated_total:
            lowest = node
    return lowest

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

# A* to find best path from start to goal, lecture 15 as reference.
def pathfindAstar(graph, start, goal):
    
    # Used to keep track of needed information for each node.
    # Have to reset this, or else infinite loop on future iterations!
    for node in graph.nodes:
        node.status = 0
        node.cost_so_far = 0
        node.estimated_total = 0
        node.previous_node = 0
    
    startRecord = start
    startRecord.estimated_heuristic = cal_heuristic(start, goal)
    startRecord.estimated_total = startRecord.cost_so_far + startRecord.estimated_heuristic

    open = [startRecord]
    closed = []

    while len(open) > 0:                        # Iterate over each node.
        current = find_lowest(open)             # Find node with smallest total cost.
        
        if current.number == goal.number:       # Exit loop when goal node is smallest.
            break
        
        connections = graph.get_connections(current)    # Get current node's connections

        for (toNode, cost) in connections:              # Loop over current node's connections.

            toNodeCost = current.cost_so_far + cost
            
            closed_node = 0                                     # Check if the node has been closed.
            for node in closed:
                if node.number == toNode.number:
                    closed_node = node
                    break
            
            if closed_node != 0:                                # If our node is closed, check the paths.
                if  closed_node.cost_so_far <= toNodeCost:
                    continue                                    # New path isn't better, skip!!
                closed.remove(closed_node)                      # New path is better, remove from closed.
                toNodeHeuristic = closed_node.estimated_total - closed_node.cost_so_far
            
            else:

                open_node = 0
                for node in open:                               # If our node isn't closed, we check if it's opened.     
                    if node.number == toNode.number:
                        open_node = node
                        break
                if open_node != 0:                              # If our node is open, check the paths.
                    if open_node.cost_so_far <= toNodeCost:
                        continue                                # New path isn't better, skip! 
                    toNodeRecord = open_node                    # New path is better, update the previous node to point to the better one.
                    toNodeRecord.previous_node = current.number
                    toNodeHeuristic = toNodeRecord.estimated_total - toNodeRecord.cost_so_far
                else:                                           # To node is unvisited.
                    toNodeRecord = toNode                       # Use node record for unvisited node.
                    toNodeRecord.previous_node = current.number 
                    toNodeHeuristic = cal_heuristic(toNode, goal)

            toNodeRecord.cost_so_far = toNodeCost               # Update node record
            toNodeRecord.estimated_total = toNodeCost + toNodeHeuristic
            if toNodeRecord not in open:
                open.append(toNodeRecord)

        open.remove(current)            # Processing current node completed!
        closed.append(current)

    if current.number != goal.number:   # Check if we reached the goal node.
        print("Path not found!\n")
        return([], 0)                   # If goal can't be reached, return empty path & final cost.

    final_cost = current.cost_so_far    # Store the final cost of traversal for printing.

    path = []                           # Assemble our path, working backwards and following previous nodes.
    while current.number != start.number:               
        path.append(current.number)
        current = graph.nodes[current.previous_node - 1]
    path.append(start.number)
    path.reverse()

    return (path, final_cost)

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
    
    # Main loop, set start and goal nodes & calculate the sorted paths & final cost.
    for start_num, goal_num in paths:
        for node in graph.nodes:
            if node.number == start_num:
                start_node = node
            if node.number == goal_num:
                goal_node = node
        
        # Use our A* function to find the final sorted path & cost.
        path, cost = pathfindAstar(graph, start_node, goal_node)
    
        file.write(f"Path from {start_num} to {goal_num} = {path}, cost = {cost}\n")