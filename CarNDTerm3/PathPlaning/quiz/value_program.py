# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0],
        [0, 0, 0, 1, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']


def optimum_policy(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    delta_name2 = ['v', '>', '^', '<']
    x, y = goal
    value = [[99 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    path = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]


    open = [[x, y]]
    closed[x][y] = 1
    value[x][y] = 0
    path[x][y] = '*'
    completed = False

    while not completed:
        next = open.pop()
        x = next[0]
        y = next[1]
        for d in delta:
            x2 = x + d[0]
            y2 = y + d[1]
            if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                if closed[x2][y2] == 1 or grid[x2][y2] == 1:
                   continue
                open.append([x2, y2])            
        # Add its neigbours into the list
        if closed[x][y] != 1:
            cur_value = 99
            camefrom_action = -1
            for i, d in enumerate(delta):
                x2 = x - d[0]
                y2 = y - d[1]
                # import pdb; pdb.set_trace()
                if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                    if value[x2][y2] != 99:
                        cur_value = min(cur_value, value[x2][y2] + cost)
                        path[x][y] = delta_name2[i]
            # import pdb; pdb.set_trace()
            value[x][y] = cur_value
            closed[x][y] = 1
        if len(open) == 0:
            completed = True
        # print(value)
    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    return path 


expand = compute_value(grid,goal,cost)
for e in expand:
    print(e)
