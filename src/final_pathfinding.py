
from queue import PriorityQueue
import json
class Pathfinder():
    def __init__(self,start, draw, obstacle):
        self.start=start
        self.draw=draw
        self.obstacle=obstacle

    def counterclockwise(self, A,B,C):
        return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

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
            i = i + 1
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


    def all_together_now(self):
        starts = [[self.start, self.start]]
        all_nodes = starts+self.draw
        number_of_nodes = len(all_nodes)
        g = self.Graph(number_of_nodes)
        i = 0
        while i < number_of_nodes:
            print('i=%s nodes=%s' % (i, number_of_nodes))
            j=i+1
            while j < number_of_nodes:
                print('j=%s nodes=%s' % (j, number_of_nodes))
                g.add_edge(i,j, self.total_dist(i,j, all_nodes, self.obstacle))
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
