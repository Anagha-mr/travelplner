
import networkx as nx
import matplotlib.pyplot as plt
import heapq

class Graph:
    def __init__(self):
        self.locations = {}

    def add_location(self, location):
        if location not in self.locations:
            self.locations[location] = []

    def add_route(self, from_location, to_location, distance):
        self.add_location(from_location)
        self.add_location(to_location)
        self.locations[from_location].append((to_location, distance))
        self.locations[to_location].append((from_location, distance))  # Assuming undirected graph

    def display_routes(self):
        for location, routes in self.locations.items():
            print(f"{location}: " + ", ".join([f"{dest} ({dist} km)" for dest, dist in routes]))

    def find_shortest_route(self, start, end):
        # Dijkstra's algorithm
        priority_queue = [(0, start, [])]  # (current_distance, current_location, path)
        visited = set()

        while priority_queue:
            current_distance, current_location, path = heapq.heappop(priority_queue)

            if current_location in visited:
                continue

            visited.add(current_location)
            path = path + [current_location]

            if current_location == end:
                return path, current_distance

            for neighbor, distance in self.locations.get(current_location, []):
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (current_distance + distance, neighbor, path))

        return None, float('inf')  # If no route is found

    def visualize_graph(self):
        G = nx.Graph()

        for location, routes in self.locations.items():
            for neighbor, distance in routes:
                G.add_edge(location, neighbor, weight=distance)

        pos = nx.spring_layout(G)  # Positioning of nodes
        weights = nx.get_edge_attributes(G, 'weight')

        nx.draw(G, pos, with_labels=True, node_size=2000, node_color='lightblue', font_size=10)
        nx.draw_networkx_edge_labels(G, pos, edge_labels=weights)
        plt.title("Travel Planner Graph")
        plt.show()

# Example usage
travel_planner = Graph()

# Adding locations and routes
n = int(input("Enter number of places (destinations): "))
for _ in range(n):
    from_loc = input("From location: ")
    to_loc = input("To location: ")
    dist = int(input("Distance (in km): "))
    travel_planner.add_route(from_loc, to_loc, dist)

# Display all routes
travel_planner.display_routes()

# Visualize the graph
travel_planner.visualize_graph()

# Find the shortest route from start to end
start = input("Enter start location: ")
end = input("Enter end location: ")
route, distance = travel_planner.find_shortest_route(start, end)
if route:
    print(f"Shortest route from {start} to {end}: {' -> '.join(route)} with distance {distance} km")
else:
    print(f"No route found from {start} to {end}.")
