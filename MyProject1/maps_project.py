import googlemaps
import networkx as nx
import heapq
from gmplot import gmplot
import tkinter as tk
from tkinter import ttk

class RouteFinderApp:
    def __init__(self, master):
        self.master = master
        master.title("Route Finder")

        self.api_key_label = tk.Label(master, text="Google Maps API Key:")
        self.api_key_entry = tk.Entry(master)
        self.origin_label = tk.Label(master, text="Origin City:")
        self.origin_entry = tk.Entry(master)
        self.destination_label = tk.Label(master, text="Destination City:")
        self.destination_entry = tk.Entry(master)
        self.submit_button = tk.Button(master, text="Find Route", command=self.find_route)

        self.api_key_label.pack(pady=5)
        self.api_key_entry.pack(pady=5)
        self.origin_label.pack(pady=5)
        self.origin_entry.pack(pady=5)
        self.destination_label.pack(pady=5)
        self.destination_entry.pack(pady=5)
        self.submit_button.pack(pady=10)

    def find_route(self):
        api_key = self.api_key_entry.get()
        origin_city = self.origin_entry.get()
        destination_city = self.destination_entry.get()

        directions = get_google_maps_directions(api_key, origin_city, destination_city)
        graph = build_graph(directions)

        shortest_path = a_star_search(graph, tuple(directions[0]['start_location'].values()),
                                      tuple(directions[-1]['end_location'].values()))

        print("Shortest Path:")
        print(shortest_path)

        visualize_route_on_map(origin_city, destination_city, directions, shortest_path)


def get_google_maps_directions(api_key, origin, destination):
    gmaps = googlemaps.Client(key=api_key)
    directions_result = gmaps.directions(origin, destination, mode="driving")
    return directions_result[0]['legs'][0]['steps']

def heuristic(node, goal):
    return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

def a_star_search(graph, start, goal):
    open_set = [(0, start)]
    closed_set = set()
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = [current]
            while current in g_score and g_score[current] > 0:
                current = min(graph.neighbors(current), key=lambda x: g_score[x] + heuristic(x, goal))
                path.insert(0, current)
            return path

        closed_set.add(current)

        for neighbor in graph.neighbors(current):
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score.get(current, float('inf')) + graph[current][neighbor]['weight']

            if neighbor not in [i[1] for i in open_set] or tentative_g_score < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

def build_graph(directions):
    G = nx.Graph()
    for step in directions:
        start_point = (step['start_location']['lat'], step['start_location']['lng'])
        end_point = (step['end_location']['lat'], step['end_location']['lng'])
        distance = step['distance']['value']  # in meters
        G.add_edge(start_point, end_point, weight=distance)
    return G

def visualize_route_on_map(origin, destination, directions, path):
    start_point = (directions[0]['start_location']['lat'], directions[0]['start_location']['lng'])
    end_point = (directions[-1]['end_location']['lat'], directions[-1]['end_location']['lng'])

    gmap = gmplot.GoogleMapPlotter(start_point[0], start_point[1], 10)

    gmap.marker(start_point[0], start_point[1], title=f"Origin: {origin}")
    gmap.marker(end_point[0], end_point[1], title=f"Destination: {destination}")

    lats, lngs = zip(*path)
    gmap.plot(lats, lngs, 'blue', edge_width=5)

    # Display distance on the map
    total_distance = sum(step['distance']['value'] for step in directions)
    gmap.text(start_point[0], start_point[1], f"Total Distance: {total_distance} meters", color='black')

    gmap.draw("route_map.html")

def main():
    root = tk.Tk()
    app = RouteFinderApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
