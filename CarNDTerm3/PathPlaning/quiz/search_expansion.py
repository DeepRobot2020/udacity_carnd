# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space
import heapq

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid, init, goal, cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    open_list = [] # priority q
    close_list = {} # dict, key is the g_value
    cur_pos = init
    g_value = 0
    max_p0, max_p1 = len(grid)-1, len(grid[0])-1
    while cur_pos[0] <= max_p0 and cur_pos[1] <= max_p1:
        # add current node into close_list
        close_list[tuple(cur_pos)] = g_value
        for d in delta:
            tmp_p0 = cur_pos[0] + d[0]
            tmp_p1 = cur_pos[1] + d[1]
            exp_pos = [tmp_p0, tmp_p1]
            if tmp_p0 < 0 or tmp_p1 < 0 or tmp_p0 > max_p0 or tmp_p1 > max_p1:
                continue
            if grid[tmp_p0][tmp_p1] == 1:
                continue
            if tuple(exp_pos) in close_list:
                continue
            exp_g_value = close_list[tuple(cur_pos)] + cost
            print('cur',cur_pos, 'exp',exp_pos, exp_g_value)
            # add expanded node and its g_value into open_list
            heapq.heappush(open_list, (exp_g_value, exp_pos))
        # update current position, which is the node with minimum g_value in open_list
        if len(open_list) == 0:
            break
        nxt_node = heapq.heappop(open_list)
        g_value = nxt_node[0]
        cur_pos = nxt_node[1]
        if cur_pos[0] == goal[0] and cur_pos[1] == goal[1]:
            return [g_value, cur_pos[0], cur_pos[1]]
    return 'fail'

print(search(grid, init, goal, cost))
