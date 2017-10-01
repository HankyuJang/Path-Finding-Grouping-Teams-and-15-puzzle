#!/usr/bin/env python
# 
# solver16.py : Solve the 16 puzzle problem - upto 3 tiles to move.
#
# (1)   
# State space: All possible formulation of the tiles in 16 puzzle problem.
# For example, a sample of the state look like this,
# S0 = array([[ 2,  3,  0,  4],
#             [ 1,  6,  7,  8],
#             [ 5, 10, 11, 12],
#             [ 9, 13, 14, 15]])
#
# Successor function: Possible position of the tiles after 1 move (either moving 1, 2, or 3 tiles at ones)
# I marked the each successor function with its appropriate move with the 3 character notation.
# 
# >>> successor(S0)
# [(array([[ 2,  3,  7,  4],
#          [ 1,  6,  0,  8],
#          [ 5, 10, 11, 12],
#          [ 9, 13, 14, 15]]), 'U11'), 
#  (array([[ 2,  3,  7,  4],
#          [ 1,  6, 11,  8],
#          [ 5, 10,  0, 12],
#          [ 9, 13, 14, 15]]), 'U21'), 
#  (array([[ 2,  3,  7,  4],
#          [ 1,  6, 11,  8],
#          [ 5, 10, 14, 12],
#          [ 9, 13,  0, 15]]), 'U31'), 
#  (array([[ 2,  0,  3,  4],
#          [ 1,  6,  7,  8],
#          [ 5, 10, 11, 12],
#          [ 9, 13, 14, 15]]), 'R13'), 
#  (array([[ 0,  2,  3,  4],
#          [ 1,  6,  7,  8],
#          [ 5, 10, 11, 12],
#          [ 9, 13, 14, 15]]), 'R23'), 
#  (array([[ 2,  3,  4,  0],
#          [ 1,  6,  7,  8],
#          [ 5, 10, 11, 12],
#          [ 9, 13, 14, 15]]), 'L13')]
# 
# Edge weights: 1 (One valid move is calculated as cost of 1)
#
# Goal state: Following is the goal state
#
# array([[ 1,  2,  3,  4],
#        [ 5,  6,  7,  8],
#        [ 9, 10, 11, 12],
#        [13, 14, 15,  0]])

# Heuristic function: (Sum of Manhattan cost) / 3
# 
# If I use the sum of the Manhattan cost as in the notes, it would be not admissble due to the over-estimating.
# I can move the tiles upto 3, which means that Dividing the sum of Manhattan cost by 3 won't over-estimate.
# Hence, this huristic function is admissible.
# Also, it is consistent, because it meets the triangle inequality.
#
# (2) How the search algorithm work
#
# For each step, the algorithm chooses to branch the node with the minimum f value, which is (heuristic + cost). It keeps branching until it reaches the goal state.
#
# (3) Any problem I faced, assumptions, simplifications, design decisions
#
# I didn't make any assumptions or simplifications. 
#

from __future__ import division
import sys
import numpy as np
from scipy.spatial.distance import cdist

n_tile = range(4)
col_move = {0:["L11","L21","L31"],1:["R12","L12","L22"],2:["R13","R23","L13"],3:["R14","R24","R34"]}
row_move = {0:["U11","U21","U31"],1:["D12","U12","U22"],2:["D13","D23","U13"],3:["D14","D24","D34"]}

G = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,0]])

# Save the position to dictionary
# Each dictionary holds the row, col index as value in a numpy array
# Then, returns the sum of the mahattan distances per all tiles divided by 3.
def manhattan_distance(s1, s2):
    s1_dict = {}
    s2_dict = {}
    for i in n_tile:
        for j in n_tile:
            s1_dict[s1[i][j]] = np.array([i, j])
            s2_dict[s2[i][j]] = np.array([i, j])
    return sum([np.abs(s1_dict[key]-s2_dict[key]).sum() for key in s1_dict]) / 3

def initial_state(filename):
    file = open(filename, "r")
    return np.array([[int(tile) for tile in line.split()] for line in file])
    
def swap(s, r1, c1, r2, c2):
    temp = s[r1,c1]
    s[r1,c1] = s[r2,c2]
    s[r2,c2] = temp

def successor(s):
    successor_list = []
    # Get the index of the blank tile
    row, col = np.where(s==0)
    row_idx, col_idx = row[0], col[0]
    # Get the 6 possible moves
    possible_move = row_move[row_idx] + col_move[col_idx]
    for move in possible_move:
        row, col = row_idx, col_idx
        s_next = np.copy(s)
        # Get direction and the number of tiles to move
        direction, n, _ = move
        n = int(n)
        if direction=='D':
            for i in range(n):
                swap(s_next,row,col,row-1,col)
                row -= 1
        elif direction=='U':
            for i in range(n):
                swap(s_next,row,col,row+1,col)
                row += 1
        elif direction=='R':
            for i in range(n):
                swap(s_next,row,col,row,col-1)
                col -= 1
        elif direction=='L':
            for i in range(n):
                swap(s_next,row,col,row,col+1)
                col += 1
        successor_list.append((s_next, move))

    return successor_list
    
def is_goal(s):
    return (s == G).all()

def heuristic(s1, s2):
    return manhattan_distance(s1, s2)

# f = heuristic + cost so far
def find_best_state(fringe):
    f_list = [s[0] + len(s[1]) for s in fringe]
    return f_list.index(min(f_list))

def solve(initial_board):
    global puzzle_tracking
    fringe = [[heuristic(initial_board, G), [], initial_board]]
    while len(fringe) > 0:
        # pop the tile with minimum value heuristic
        _, move_upto, s = fringe.pop(find_best_state(fringe))
        # _, s = fringe.pop(0) # this is using BFS
        for s_prime, move in successor(s):
            if is_goal(s_prime):
                return (fringe, s_prime, move_upto+[move])
            fringe.append([heuristic(s_prime, G), move_upto+[move], s_prime])
    return False

def printable_result(path):
    return " ".join(path)

filename = sys.argv[1]
S0 = initial_state(filename)

fringe, s_prime, path = solve(S0)
print printable_result(path)
