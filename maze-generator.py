# Maze generator -- Randomized Prim Algorithm

## Imports
import random
import time
from colorama import init
from colorama import Fore, Back, Style

## Functions
def printMaze(maze):
    for i in range(0, HEIGHT):
        for j in range(0, WIDTH):
            if (maze[i][j] == UNVISITED):
                print(Fore.WHITE + str(maze[i][j]), end=" ")
            elif (maze[i][j] == CELL):
                print(Fore.GREEN + str(maze[i][j]), end=" ")
            else:
                print(Fore.RED + str(maze[i][j]), end=" ")
            
        print('\n')

# Find number of surrounding cells
def surroundingCells(rand_wall):
    s_cells = 0
    if (maze[rand_wall[0]-1][rand_wall[1]] == CELL):
        s_cells += 1
    if (maze[rand_wall[0]+1][rand_wall[1]] == CELL):
        s_cells += 1
    if (maze[rand_wall[0]][rand_wall[1]-1] == CELL):
        s_cells +=1
    if (maze[rand_wall[0]][rand_wall[1]+1] == CELL):
        s_cells += 1

    return s_cells


## Main code
# Init variables
WALL = '+'
CELL = ' '
UNVISITED = 'u'
HEIGHT = 10
WIDTH = 65
maze = []

# Initialize colorama
init()

# Denote all cells as UNVISITED
for i in range(0, HEIGHT):
    line = []
    for j in range(0, WIDTH):
        line.append(UNVISITED)
    maze.append(line)

# Randomize starting point and set it a cell
starting_height = random.randint(1, HEIGHT-2)
starting_width = random.randint(1, WIDTH-2)

# Mark it as cell and add surrounding walls to the list
maze[starting_height][starting_width] = CELL
walls = []
walls.append([starting_height - 1, starting_width])
walls.append([starting_height, starting_width - 1])
walls.append([starting_height, starting_width + 1])
walls.append([starting_height + 1, starting_width])

print(walls)

# Denote walls in maze
maze[starting_height-1][starting_width] = WALL
maze[starting_height][starting_width - 1] = WALL
maze[starting_height][starting_width + 1] = WALL
maze[starting_height + 1][starting_width] = WALL

while (walls):
    # Pick a random wall
    rand_wall = walls[int(random.random()*len(walls))-1]

    # Check if it is a left wall
    if (rand_wall[1] != 0):
        if (maze[rand_wall[0]][rand_wall[1]-1] == UNVISITED and maze[rand_wall[0]][rand_wall[1]+1] == CELL):
            # Find the number of surrounding cells
            s_cells = surroundingCells(rand_wall)

            if (s_cells < 2):
                # Denote the new path
                maze[rand_wall[0]][rand_wall[1]] = CELL

                # Mark the new walls
                # Upper cell
                if (rand_wall[0] != 0):
                    if (maze[rand_wall[0]-1][rand_wall[1]] != CELL):
                        maze[rand_wall[0]-1][rand_wall[1]] = WALL
                    if ([rand_wall[0]-1, rand_wall[1]] not in walls):
                        walls.append([rand_wall[0]-1, rand_wall[1]])


                # Bottom cell
                if (rand_wall[0] != HEIGHT-1):
                    if (maze[rand_wall[0]+1][rand_wall[1]] != CELL):
                        maze[rand_wall[0]+1][rand_wall[1]] = WALL
                    if ([rand_wall[0]+1, rand_wall[1]] not in walls):
                        walls.append([rand_wall[0]+1, rand_wall[1]])

                # Leftmost cell
                if (rand_wall[1] != 0):    
                    if (maze[rand_wall[0]][rand_wall[1]-1] != CELL):
                        maze[rand_wall[0]][rand_wall[1]-1] = WALL
                    if ([rand_wall[0], rand_wall[1]-1] not in walls):
                        walls.append([rand_wall[0], rand_wall[1]-1])
            

            # Delete wall
            for wall in walls:
                if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                    walls.remove(wall)

            continue

    # Check if it is an upper wall
    if (rand_wall[0] != 0):
        if (maze[rand_wall[0]-1][rand_wall[1]] == UNVISITED and maze[rand_wall[0]+1][rand_wall[1]] == CELL):

            s_cells = surroundingCells(rand_wall)
            if (s_cells < 2):
                # Denote the new path
                maze[rand_wall[0]][rand_wall[1]] = CELL

                # Mark the new walls
                # Upper cell
                if (rand_wall[0] != 0):
                    if (maze[rand_wall[0]-1][rand_wall[1]] != CELL):
                        maze[rand_wall[0]-1][rand_wall[1]] = WALL
                    if ([rand_wall[0]-1, rand_wall[1]] not in walls):
                        walls.append([rand_wall[0]-1, rand_wall[1]])

                # Leftmost cell
                if (rand_wall[1] != 0):
                    if (maze[rand_wall[0]][rand_wall[1]-1] != CELL):
                        maze[rand_wall[0]][rand_wall[1]-1] = WALL
                    if ([rand_wall[0], rand_wall[1]-1] not in walls):
                        walls.append([rand_wall[0], rand_wall[1]-1])

                # Rightmost cell
                if (rand_wall[1] != WIDTH-1):
                    if (maze[rand_wall[0]][rand_wall[1]+1] != CELL):
                        maze[rand_wall[0]][rand_wall[1]+1] = WALL
                    if ([rand_wall[0], rand_wall[1]+1] not in walls):
                        walls.append([rand_wall[0], rand_wall[1]+1])

            # Delete wall
            for wall in walls:
                if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                    walls.remove(wall)

            continue

    # Check the bottom wall
    if (rand_wall[0] != HEIGHT-1):
        if (maze[rand_wall[0]+1][rand_wall[1]] == UNVISITED and maze[rand_wall[0]-1][rand_wall[1]] == CELL):

            s_cells = surroundingCells(rand_wall)
            if (s_cells < 2):
                # Denote the new path
                maze[rand_wall[0]][rand_wall[1]] = CELL

                # Mark the new walls
                if (rand_wall[0] != HEIGHT-1):
                    if (maze[rand_wall[0]+1][rand_wall[1]] != CELL):
                        maze[rand_wall[0]+1][rand_wall[1]] = WALL
                    if ([rand_wall[0]+1, rand_wall[1]] not in walls):
                        walls.append([rand_wall[0]+1, rand_wall[1]])
                if (rand_wall[1] != 0):
                    if (maze[rand_wall[0]][rand_wall[1]-1] != CELL):
                        maze[rand_wall[0]][rand_wall[1]-1] = WALL
                    if ([rand_wall[0], rand_wall[1]-1] not in walls):
                        walls.append([rand_wall[0], rand_wall[1]-1])
                if (rand_wall[1] != WIDTH-1):
                    if (maze[rand_wall[0]][rand_wall[1]+1] != CELL):
                        maze[rand_wall[0]][rand_wall[1]+1] = WALL
                    if ([rand_wall[0], rand_wall[1]+1] not in walls):
                        walls.append([rand_wall[0], rand_wall[1]+1])

            # Delete wall
            for wall in walls:
                if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                    walls.remove(wall)


            continue

    # Check the right wall
    if (rand_wall[1] != WIDTH-1):
        if (maze[rand_wall[0]][rand_wall[1]+1] == UNVISITED and maze[rand_wall[0]][rand_wall[1]-1] == CELL):

            s_cells = surroundingCells(rand_wall)
            if (s_cells < 2):
                # Denote the new path
                maze[rand_wall[0]][rand_wall[1]] = CELL

                # Mark the new walls
                if (rand_wall[1] != WIDTH-1):
                    if (maze[rand_wall[0]][rand_wall[1]+1] != CELL):
                        maze[rand_wall[0]][rand_wall[1]+1] = WALL
                    if ([rand_wall[0], rand_wall[1]+1] not in walls):
                        walls.append([rand_wall[0], rand_wall[1]+1])
                if (rand_wall[0] != HEIGHT-1):
                    if (maze[rand_wall[0]+1][rand_wall[1]] != CELL):
                        maze[rand_wall[0]+1][rand_wall[1]] = WALL
                    if ([rand_wall[0]+1, rand_wall[1]] not in walls):
                        walls.append([rand_wall[0]+1, rand_wall[1]])
                if (rand_wall[0] != 0):    
                    if (maze[rand_wall[0]-1][rand_wall[1]] != CELL):
                        maze[rand_wall[0]-1][rand_wall[1]] = WALL
                    if ([rand_wall[0]-1, rand_wall[1]] not in walls):
                        walls.append([rand_wall[0]-1, rand_wall[1]])

            # Delete wall
            for wall in walls:
                if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                    walls.remove(wall)

            continue

    # Delete the wall from the list anyway
    for wall in walls:
        if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
            walls.remove(wall)
    


# Mark the remaining UNVISITED cells as walls
for i in range(0, HEIGHT):
    for j in range(0, WIDTH):
        if (maze[i][j] == UNVISITED):
            maze[i][j] = WALL

# Print final maze
printMaze(maze)