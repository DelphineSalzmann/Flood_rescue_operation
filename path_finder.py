
#implementação de Astar e wavefront expansion para ver qual é o melhor

from queue import PriorityQueue
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def manhattan_heuristic(xy1, xy2):
    return abs(xy1[0]-xy2[0]) + abs(xy1[1]- xy2[1])
#function able to build the grid with the correct mapping coverting real to the grid
def build_grid(poses, base, obstacles = []):
    x_min, x_max = -2.0, 2.0
    y_min, y_max = -4.0, 4.0

    resolution = 0.2
    
    x_size = int((x_max - x_min) / resolution)+1
    y_size = int((y_max - y_min) / resolution)+1
    grid = np.zeros((y_size, x_size))

    for i, pose in enumerate(poses):
        y_idx, x_idx = real_to_grid(pose[0], pose[1])
        if i == 0:
            grid[y_idx, x_idx] = 3  # start
        else:
            grid[y_idx, x_idx] = 1  # victmin
    for obstacle in obstacles:
        y_idx, x_idx = real_to_grid(obstacle[0], obstacle[1])
        grid[y_idx, x_idx] = -1 # obstacles
        sucessors = get_successors((y_idx, x_idx), grid)
        for sucessor in sucessors:
            if sucessor[1] < y_size and sucessor[0] < x_size:
                print('sucessor', sucessor)
                y_id, x_id = real_to_grid(sucessor[0], sucessor[1])
                grid[y_id, x_id] = -1 # obstacles


    y_idx, x_idx = real_to_grid(base[0], base[1])
    grid[y_idx, x_idx] = 2 #base
    return grid
    # place the firefighter and the victmins in the grid

# aux function to turn the number in the grid
def real_to_grid(x, y, x_min =-2.0, y_min = -4.0, resolution = 0.2):
    resolution = 0.2
    x_idx = int((x - x_min) / resolution)
    y_idx = int((y - y_min) / resolution)
    return y_idx, x_idx
#aux invert
def grid_to_real(points, x_min=-2.0, y_min=-4.0, resolution=0.2):
    transformed_points = []
    for y_idx, x_idx in points:
        x = x_idx * resolution + x_min
        y = y_idx * resolution + y_min
        transformed_points.append((x, y))
    return transformed_points

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
    print(start, victims, base, obstacle)
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

#not supose to be use outside this script
def plot_grid(grid, path= None):
    colors = {0: 'white', -1: 'black', 1: 'red', 2: 'blue', 3: 'green'}
    colored_grid = np.zeros(grid.shape + (3,), dtype=np.uint8)

    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            color = colors[grid[r, c]]
            colored_grid[r, c] = np.array(plt.cm.colors.to_rgba(color)[:3]) * 255

    plt.imshow(colored_grid)
    plt.title('Grid with start (blue), victims (red)and base (green). Path is yellow')
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, color='yellow')  # Plotting the path in yellow
    def update(frame):
        if frame < len(path):
            # Plot the path up to the current frame
            path_x, path_y = zip(*path[:frame+1])
            plt.plot(path_y, path_x, color='yellow')
            plt.scatter(path_y[-1], path_x[-1], color='red')  # Highlight the current position with a red dot
        else:
            # If all frames have been shown, plot the entire path
            path_x, path_y = zip(*path)
            plt.plot(path_y, path_x, color='yellow')
        plt.xlabel('Y')
        plt.ylabel('X')
        plt.title('Animated Path')     
    fig = plt.gcf()
    ani = FuncAnimation(fig, update, frames=len(path)+1, interval=100, repeat=False) 
    plt.show()



if __name__ == "__main__":
    grid = build_grid([(0,0), (-1, 0.4), (-0.4, -1), (2, 0,4)], (0,0.2), [(1,1)])
    plot_grid(grid)

    path = path_maker(grid)
    print(path)
    plot_grid(grid, path)

    path = grid_to_real(path)
    print(path)