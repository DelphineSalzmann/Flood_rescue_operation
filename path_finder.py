
#implementação de Astar e wavefront expansion para ver qual é o melhor

from queue import PriorityQueue


def manhattan_heuristic(xy1, xy2):
    return abs(xy1[0]-xy2[0]) + abs(xy1[1]- xy2[1])

def get_successors(point, grid):
    list_successors = []
    rows = len(grid)
    cols = len(grid[0])
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]  # Up, Down, Left, Right and diagonals
    for dx, dy in offsets:
        successor = (point[0] + dx, point[1] + dy)
        if 0 <= successor[0] < rows and 0 <= successor[1] < cols and grid[successor[0]][successor[1]] != -1: #eliminated the part that is not in the grid and the obstacles
            list_successors.append(successor)
    return list_successors

def is_obstacle(point, grid):
     return grid[point[0]][point[1]] == -1

def is_victim(point, grid):
     return grid[point[0]][point[1]] == 1

def is_base(point, grid):
     return grid[point[0]][point[1]] == 2

def is_start(point, grid):
     return grid[point[0]][point[1]] == 3
#get the victims and return to base

def is_objective(point, objective):
    return point == objective

def astar(grid, start, objective):
    queue = PriorityQueue()
    queue.put((0, (start, [])))
    visited = set()
    while not queue.empty():
        current_cost, item = queue.get()

        if is_objective(item[0], objective): # Assuming only one start point
            return item[1]
        if item[0] not in visited:
            visited.add(item[0])
            for sucessor in get_successors(item[0], grid):
                if sucessor not in visited:
                    queue.put((manhattan_heuristic(sucessor,objective)+current_cost, (sucessor, item[1] + [sucessor] )))            

    return []

def path_maker(grid):
    start = []
    victims = []
    base = []
    obstacle = []
    path_total = []
    
    # Find start, victims, and base positions
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if is_start((i, j), grid):
                start.append((i, j))
            if is_victim((i, j), grid):
                victims.append((i, j))
            if is_base((i, j), grid):
                base.append((i, j))
            if is_obstacle((i,j), grid):
                obstacle.append((i,j))

    
    # A* to get to the first victim
    #print(obstacle)
    path = astar(grid, start[0], victims[0])
    path_total.extend(path)
    
    # A* to get from each victim to the next one
    for i in range(len(victims) - 1):
        path = astar(grid, victims[i], victims[i+1])
        path_total.extend(path)
    
    # A* to get from the last victim to the base
    path = astar(grid, victims[-1], base[0])
    path_total.extend(path)
    
    return path_total

    



if __name__ == "__main__":
    grid = [[0 for _ in range(6)] for _ in range(8)]
    grid[0][0] = 3  # Start
    grid[0][1] = 2  # base

    #victimins
    grid[3][1] = 1
    grid[4][5] = 1
    grid[7][5] = 1

    #obstacles
    #grid[2][1] = -1
    #grid[2][2] = -1
    #grid[3][3] = -1
    #grid[5][2] = -1
    def print_grid(grid):
        for row in grid:
            print(" ".join(str(cell) for cell in row))

    print_grid(grid)

    path = path_maker(grid)
    print("Caminho A*:", path)