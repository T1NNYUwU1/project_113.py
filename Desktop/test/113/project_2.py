import time
# ส่วนที่ 0: เอามาหาเวลาในการรัน

class Graph:
    def __init__(self, vertices):
        # ส่วนที่ 1: สร้างกราฟโดยกำหนดจำนวน vertices และ adjacency matrix
        self.vertices = vertices
        self.adj_matrix = [[0] * vertices for _ in range(vertices)]

    def add_edge(self, start, end, weight):
        # ส่วนที่ 2: เพิ่ม edge และ weight ใน adjacency matrix
        self.adj_matrix[start][end] = weight
        self.adj_matrix[end][start] = weight

    def dfs_traversal(self, start):
        # ส่วนที่ 3: DFS Traversal
        visited = [False] * self.vertices
        result = []

        def dfs(vertex):
            visited[vertex] = True
            result.append(vertex)
            for i in range(self.vertices):
                if self.adj_matrix[vertex][i] > 0 and not visited[i]:
                    dfs(i)

        dfs(start)
        return result

    def dijkstra(self, start, end):
        # ส่วนที่ 4: Dijkstra's Algorithm
        INF = float('inf')
        distances = [INF] * self.vertices
        distances[start] = 0
        visited = [False] * self.vertices
        parent = [-1] * self.vertices

        for _ in range(self.vertices):
            min_dist = INF
            for i in range(self.vertices):
                if distances[i] < min_dist and not visited[i]:
                    min_dist = distances[i]
                    current_vertex = i

            visited[current_vertex] = True

            for i in range(self.vertices):
                if (
                    self.adj_matrix[current_vertex][i] > 0
                    and not visited[i]
                    and distances[i] > distances[current_vertex] + self.adj_matrix[current_vertex][i]
                ):
                    distances[i] = distances[current_vertex] + self.adj_matrix[current_vertex][i]
                    parent[i] = current_vertex

        path = []
        current_vertex = end
        while current_vertex != -1:
            path.insert(0, current_vertex)
            current_vertex = parent[current_vertex]

        return distances[end], path

    def prim(self):
        # ส่วนที่ 5: Prim's Algorithm
        INF = float('inf')
        key = [INF] * self.vertices
        parent = [-1] * self.vertices
        mst_set = [False] * self.vertices

        key[0] = 0

        for _ in range(self.vertices):
            min_key = INF
            for i in range(self.vertices):
                if key[i] < min_key and not mst_set[i]:
                    min_key = key[i]
                    u = i

            mst_set[u] = True

            for v in range(self.vertices):
                if self.adj_matrix[u][v] > 0 and not mst_set[v] and key[v] > self.adj_matrix[u][v]:
                    key[v] = self.adj_matrix[u][v]
                    parent[v] = u

        return parent

    def print_mst(self, parent):
        # ส่วนที่ 6: แสดงผล Minimum Spanning Tree (MST)
        print("Minimum Spanning Tree (Prim's algorithm):")
        total_weight = 0
        for i in range(1, self.vertices):
            total_weight += self.adj_matrix[i][parent[i]]
            print(f"Edge: {parent[i]} - {i}  Weight: {self.adj_matrix[i][parent[i]]}")
        print(f"Total Weight: {total_weight}")


# ส่วนที่ 7: การใช้งาน
if __name__ == "__main__":
    N = int(input("Enter the number of vertices (N): "))
    start, end = map(int, input("Enter the starting and stopping vertex: ").split())

    graph = Graph(N)

    print("Enter the adjacency matrix:")
    for i in range(N):
        row = list(map(int, input().split()))
        for j in range(N):
            graph.add_edge(i, j, row[j])

    start_time = time.time()
    # บรรทัดนี้วัดเวลาที่เริ่มต้นการรัน
    
    # ส่วนที่ 8: DFS Traversal
    dfs_result = graph.dfs_traversal(start)
    print(f"DFS Traversal: {' '.join(map(str, dfs_result))}")

    # ส่วนที่ 9: Dijkstra's Algorithm
    shortest_distance, dijkstra_path = graph.dijkstra(start, end)
    print(f"Dijkstra's Algorithm: shortest distance = {shortest_distance}")
    print(f"Order path from vertex {start} to vertex {end}: {' --> '.join([f'{node}({graph.adj_matrix[dijkstra_path[i]][dijkstra_path[i+1]]})' for i, node in enumerate(dijkstra_path[:-1])])}")

    # ส่วนที่ 10: Prim's Algorithm
    prim_result = graph.prim()
    graph.print_mst(prim_result)

    # ส่วนที่ 11: วัดเวลาที่สิ้นสุดการรันและแสดงผลเวลาที่ใช้ทั้งหมด
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Total Execution Time: {elapsed_time} seconds")