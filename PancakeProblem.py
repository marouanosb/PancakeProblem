import heapq
from collections import deque

class PancakeProblem:
    # Representation du probleme
    def __init__(self, state):
        self.state = state
        self.goal = tuple(sorted(state))

    # Vérifier si l'état actuel est la destination
    def is_goal(self, state):
        return state == self.goal

    # Retourners les k premiers pancakes
    def flip(self, state, k):
        return tuple(reversed(state[:k])) + state[k:]

    # Expand les fils de l'état actuel
    def get_neighbors(self, state):
        neighbors = []
        for k in range(2, len(state) + 1):
            neighbors.append((self.flip(state, k), k))  # Retourne (nouvel état, action)
        return neighbors

    # H(n)
    def heuristic(self, state):
        return sum(1 for i in range(len(state) - 1) if abs(state[i] - state[i + 1]) > 1)

    # A* / f(n) = g(n) + h(n)
    def a_star(self):
        open_list = []
        heapq.heappush(open_list, (self.heuristic(self.state), 0, self.state, []))
        visited = set()

        while open_list:
            _, g, current, path = heapq.heappop(open_list)

            if self.is_goal(current):
                return path #retourne le chemin

            if current in visited:
                continue
            visited.add(current)

            for neighbor, action in self.get_neighbors(current):
                heapq.heappush(open_list, (g + 1 + self.heuristic(neighbor), g + 1, neighbor, path + [action]))

        return None  # sinon pas de solution trouvee

    # Largeur
    def bfs(self):
        queue = deque([(self.state, [])])
        visited = set()

        while queue:
            current, path = queue.popleft()

            if self.is_goal(current):
                return path #retourne le chemin

            if current in visited:
                continue
            visited.add(current)

            for neighbor, action in self.get_neighbors(current):
                queue.append((neighbor, path + [action]))

        return None  # sinon pas de solution trouvée

    # Profondeur
    def dfs(self):
        stack = [(self.state, [])]
        visited = set()
        
        while stack:
            current, path = stack.pop()

            if current == self.goal:
                return path  # retourne le chemin

            if current in visited:
                continue

            visited.add(current) 

            for neighbor, action in self.get_neighbors(current):
                if neighbor not in visited:
                    stack.append((neighbor, path + [action]))

            # profoondeur informé par h(n) pour de plus petit chemins
            # for neighbor, action in sorted(self.get_neighbors(current), key=lambda x: self.heuristic(x[0]), reverse=True):
            #    if neighbor not in visited:
            #        stack.append((neighbor, path + [action]))


        return None  # Pas de solution trouvée


    # Cout uniforme
    def uniform_cost(self):
        open_list = []
        heapq.heappush(open_list, (0, self.state, []))  # cout initial 0
        visited = set()

        while open_list:
            g, current, path = heapq.heappop(open_list)

            if self.is_goal(current):
                return path #retourne le chemin

            if current in visited:
                continue
            visited.add(current)

            for neighbor, action in self.get_neighbors(current):
                heapq.heappush(open_list, (g + 1, neighbor, path + [action]))  # Coût toujours +1

        return None  # sinon pas de solution trouvee

    def greedy(self):
        open_list = []
        heapq.heappush(open_list, (self.heuristic(self.state), self.state, []))
        visited = set()

        while open_list:
            _, current, path = heapq.heappop(open_list)

            if self.is_goal(current):
                return path
            
            if current in visited:
                continue
            visited.add(current)

            for neighbor, action in self.get_neighbors(current):
                heapq.heappush(open_list, (self.heuristic(neighbor), neighbor, path + [action]))

        return None # si on trouve pas de solution

# MAIN
problems = [
    PancakeProblem((4, 6, 2, 5, 1, 3)),
    PancakeProblem((1, 3, 7, 5, 2, 6, 4)),
    PancakeProblem((1, 7, 2, 6, 3, 5, 4)),
    PancakeProblem((1, 3, 5, 7, 9, 2, 4, 6, 8))
]

for i, problem in enumerate(problems, 1):
    print(f"Problème {i} : {problem.state}")
    print("-" * 50)

    solution = problem.bfs()
    print(f"BFS : {solution}, Cout : {len(solution) if solution else 'Aucune solution'}")

    solution = problem.dfs()
    print(f"DFS : {solution}, Cout : {len(solution) if solution else 'Aucune solution'}")

    solution = problem.uniform_cost()
    print(f"Cout-uniform : {solution}, Cout : {len(solution) if solution else 'Aucune solution'}")

    solution = problem.a_star()
    print(f"A*  : {solution}, Cout : {len(solution) if solution else 'Aucune solution'}")

    solution = problem.greedy()
    print(f"Greedy  : {solution}, Cout : {len(solution) if solution else 'Aucune solution'}")

    print("=" * 50)