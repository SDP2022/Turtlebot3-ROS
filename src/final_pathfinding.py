
from queue import PriorityQueue
import json
class Pathfinder():
    def __init__(self,start, draw, obstacle):
        self.start=start
        self.draw=draw
        self.obstacle=obstacle

    def parser_file_red(self, file_path):
        with open(file_path) as f:
            gj = json.load(f)

        red_points = []
        no_features = len(gj['features'])

        for x in range(no_features):
            features = gj['features'][x]
            if features['properties']['stroke'] == '#ff0000':
                point_one = (features['geometry']['coordinates'])[0]
                point_two = (features['geometry']['coordinates'])[1]
                no_of_points = 2 ##len(points)
                ##for t in range(no_of_points):
                red_points.append((point_one, point_two))

        return red_points


    def parser_file_blue(self, file_path):
        with open(file_path) as f:
            gj = json.load(f)
        blue_list = []
        no_features = len(gj['features'])

        for x in range(no_features):
            features = gj['features'][x]
            if features['properties']['stroke'] == '#0000ff':
                blue_list = []
                no_of_points = len((features['geometry']['coordinates'])[0])

                for t in range(no_of_points):
                    point = (features['geometry']['coordinates'])[0]
                    blue_list.append(point[0])
        return blue_list

    def counterclockwise(self, A,B,C):
        return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)

    def intersect(self, A,B,C,D):
        return self.counterclockwise(A,C,D) != self.counterclockwise(B,C,D) and self.counterclockwise(A,B,C) != self.counterclockwise(A,B,D)

    class Graph:
        def __init__(self, no_vertices):
            self.v = no_vertices
            self.edge = [[-1 for i in range(no_vertices)] for j in range(no_vertices)]
            self.visited = []
        
        def add_edge(self, u, v, weight):
            self.edge[u][v] = weight
            self.edge[v][u] = weight
        
        def dijkstra(self, start):
            D = {v:float('inf') for v in range(self.v)}
            D[start] = 0
        
            pq = PriorityQueue()
            pq.put((0, start))
        
            while not pq.empty():
                (dist, current) = pq.get()
                self.visited.append(current)
        
                for neighbour in range(self.v):
                    if self.edge[current][neighbour] != -1:
                        distance = self.edge[current][neighbour]
                        if neighbour not in self.visited:
                            old = D[neighbour]
                            new = D[current] + distance
                            if new < old:
                                pq.put((new, neighbour))
                                D[neighbour] = new
            return D

    def distance(self, x, y):
        dista = pow((y[1]-x[1]), 2) + pow((y[0]-x[0]), 2)
        return dista

    def total_dist(self, x, y, points, obstacle):
        dist = 0
        intersects = False
        number_of_obstacle_pts = len(obstacle)
        i=0
        while i < number_of_obstacle_pts-1:
            if self.intersect(points[x][1], points[y][0], obstacle[i], obstacle[i+1]):
                intersects = True
        if intersects == True:
            minimum_x = 999999999
            minimum_y = 999999999
            for corner in obstacle:
                new_dist_x = self.distance(points[x][1], corner)
                new_dist_y = self.distance(points[y][0], corner)
                if new_dist_x < minimum_x:
                    minimum_x = new_dist_x
                    closest_x = corner
                if new_dist_y < minimum_y:
                    minimum_y = new_dist_y
                    closest_y = corner
            dist = self.distance(points[x][1], closest_x) + self.distance(closest_x, closest_y) + distance(closest_y, points[y][0]) + distance(points[y][0], points[y][1])
        else:
            dist = self.distance(points[x][1], points[y][0]) + self.distance(points[y][0], points[y][1])
    
        return dist


    def all_together_now(self, start, draw, obstacle):
        starts = [[start, start]]
        all_nodes = starts+draw
        number_of_nodes = len(all_nodes)
        g = self.Graph(number_of_nodes)
        i = 0
        while i < number_of_nodes:
            j=i+1
            while j < number_of_nodes:
                g.add_edge(i,j, self.total_dist(i,j, all_nodes, obstacle))
                j+=1
            i+=1
        D = g.dijkstra(0)
        order=[]
        coords_order=[]
        for w in sorted(D, key=D.get):
            order.append(w)
            coords_order.append(all_nodes[w])
        print(order)
        return coords_order
