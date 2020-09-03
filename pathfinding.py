import heapq
import copy

#represents a single point on our grid
class grid_node:
    def __init__(self, position, t):
        self.heuristic_cost = 0                         #the cost of travelling in this direction as calculated through heuristic
        self.actual_cost = 0                            #the cost from the start point

        self.type = t if t == '_' or t == 'X' else '_'  #the value of the given node, navigable or not
        self.parent = None                              #the parent node, used for pricing and reversing for the route
        self.visited_flag = 0                           #a flag to the state of the current node 0 being new, 1 being open and 2 being closed
        self.position = position                        #a representation of the current nodes xy coordinate.

    def __eq__(self, other):
        return self.position[0] == other.position[0] and self.position[1] == other.position[1]

    def __lt__(self, other):
        return (self.heuristic_cost+self.actual_cost) < (other.heuristic_cost+other.actual_cost)

#a wraper for a 2d array.
class grid:
    def __init__(self):
        self.tiles = []

class solver():
    def __init__(self, grid_stream):
        self.grid = grid_stream[0]                                             #the grid of tiles
        self.start = self.grid.tiles[grid_stream[1][0]][grid_stream[1][1]]     #start coords
        self.goal = self.grid.tiles[grid_stream[2][0]][grid_stream[2][1]]      #end coords
        self.open_list = []                                                    #queue of available tiles
        self.closed_list = []                                                  #queue of tiles that are more or less settled; due to a slight modification as suggested through the wikipedia article for A*, closed tiles may be reopened if a shorter path to them is found.


    #a search method that finds only the non-diagonal adjacent nodes to a given node
    def manhattan_search(self, node):
        base_position = node.position
        cardinals = [(base_position[0], base_position[1] - 1),
                     (base_position[0], base_position[1] + 1),
                     (base_position[0] - 1, base_position[1]),
                     (base_position[0] + 1, base_position[1])]
        search_space = []
        for c in cardinals:
            if not (min(c[0], c[1]) < 0 or c[0] >= len(self.grid.tiles) or c[1] >= len(self.grid.tiles[0])) and self.grid.tiles[c[0]][c[1]].type == '_':
                search_space.append(c)

        return search_space

    # a search method that finds all adjacent nodes to a given node
    def full_search(self, node):
        search_space = self.manhattan_search(node)
        base_position = node.position
        inter_cardinals = [(base_position[0]+1, base_position[1]+1),
                           (base_position[0]+1, base_position[1]-1),
                           (base_position[0]-1, base_position[1]+1),
                           (base_position[0]-1, base_position[1]-1)]
        for c in inter_cardinals:
            if not (min(c[0], c[1]) < 0 or c[0] >= len(self.grid.tiles) or c[1] >= len(self.grid.tiles[0])) and self.grid.tiles[c[0]][c[1]].type == '_':
                search_space.append(c)

        return search_space



    # our path finding function, takes a search function to determine how it looks for adjacent nodes, a heuristic to determine
    # what the cost of a given node is, and a flag to determine whether or not the path finder will remember the cost of arriving at a given node.
    def get_path(self, search_function, heuristic, memory):
        self.start.heuristic_cost = heuristic(self.start)
        heapq.heappush(self.open_list, (self.start))

        while len(self.open_list) > 0:
            current = heapq.heappop(self.open_list)
            if current.visited_flag == 2:           #because we might reopen a closed node, we check to see if the current found node is closed at this point
                continue
            self.closed_list.append(current)
            current.visited_flag = 2

            #have we found our goal yet?
            if current is self.goal:
                return current

            #look at the neighbours of the current node
            neighbours = search_function(current)


            for n in neighbours:
                newly_found = self.grid.tiles[n[0]][n[1]]
                if newly_found.visited_flag == 0:
                    newly_found.parent = current    #if we have never seen this node before, then its parent if the currently examined node

                if memory: #if we can actually remember the cost of getting here
                    if newly_found.actual_cost > current.actual_cost + 1:   #check if known cost to arrive at the neighbor node is higher than the cost of arriving at it through the current node.
                        newly_found.parent = current
                        newly_found.actual_cost = newly_found.parent.actual_cost + 1
                        newly_found.visited_flag = 1                        #this node is now open/

                    elif newly_found.parent is not None:
                        newly_found.actual_cost = newly_found.parent.actual_cost + 1
                #if this node was not already closed or set to closed...
                #update its heuristic cost and mark it as open.
                if newly_found.visited_flag != 2:
                    newly_found.heuristic_cost = heuristic(newly_found)
                    newly_found.visited_flag = 1
                #push this neighbor onto our queue of open nodes.
                heapq.heappush(self.open_list, newly_found)
        return None

    #heuristics
    def get_chebyshev_distance(self, target):
        return max(abs(self.goal.position[0]-target.position[0]), abs(self.goal.position[1]-target.position[1]))

    def get_manhattan_distance(self, target):
        return abs(self.goal.position[0]-target.position[0])+abs(self.goal.position[1]-target.position[1])




# 'X' is wall, '_' is clear, 'S' is start, and 'G' is goal
def read_grids(src):
    grids = []
    start = None
    goal = None
    y = 0
    g = grid()
    for line in src:
        if len(line.strip()) > 0:
            g.tiles.append([])
        x = 0
        if len(line.strip()) > 0:
            for char in line:
                if char is 'S':
                    start = (y, x)
                if char is 'G':
                    goal = (y, x)
                if char is not '\n':
                    g.tiles[y].append(grid_node((y, x), char))
                x += 1
            y += 1
        else:
            grids.append((g, start, goal))
            g = grid()
            y = 0
            start = None
            goal = None
    grids.append((g, start, goal))
    return grids


def trace_back(end_node):
    trace = []
    current = end_node
    while current is not None:
        trace.append(current.position)
        current = current.parent
    return trace


def generate_string(end_node, grid):
    path = trace_back(end_node)
    print(len(path)-1)
    for node in path:
        grid.tiles[node[0]][node[1]].type = 'P'
    grid.tiles[path[0][0]][path[0][1]].type = 'G'
    grid.tiles[path[-1][0]][path[-1][1]].type = 'S'
    map = ""
    for y in range(len(grid.tiles)):
        for x in range(len(grid.tiles[0])):
            map+=grid.tiles[y][x].type
        map+="\n"
    return map




def main():
    search_grid_file_a = open("pathfinding_a.txt", "r")
    grids_a = read_grids(search_grid_file_a)
    output_file_a = open("output_a.txt", "w")
    i = 1
    for grid in grids_a:
        print("solving grid " + str(i) + " A*")
        s1 = solver(copy.deepcopy(grid))
        q1 = s1.get_path(s1.manhattan_search, s1.get_manhattan_distance, True)
        print("solving grid " + str(i) + " Greedy")
        s2 = solver(copy.deepcopy(grid))
        q2 = s2.get_path(s2.manhattan_search, s2.get_manhattan_distance, False)
        output_file_a.write("A*\n" + generate_string(q1, s1.grid))
        output_file_a.write("Greedy\n" + generate_string(q2, s2.grid))
        i+=1
    search_grid_file_a.close()
    output_file_a.close()
    search_grid_file_b = open("pathfinding_b.txt", "r")
    grids_b = read_grids(search_grid_file_b)
    output_file_b = open("output_b.txt", "w")
    i = 1
    for grid in grids_b:
        print("solving grid " + str(i) + " A*")
        s1 = solver(copy.deepcopy(grid))
        q1 = s1.get_path(s1.full_search, s1.get_chebyshev_distance, True)
        print("solving grid " + str(i) + " Greedy")
        s2 = solver(copy.deepcopy(grid))
        q2 = s2.get_path(s2.full_search, s2.get_chebyshev_distance, False)
        output_file_b.write("A*\n" + generate_string(q1, s1.grid))
        output_file_b.write("Greedy\n" + generate_string(q2, s2.grid))
        i += 1
    search_grid_file_b.close()
    output_file_b.close()


main()