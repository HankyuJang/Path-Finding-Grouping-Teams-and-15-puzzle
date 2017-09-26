#!/usr/bin/env python
# 

import sys
import numpy as np

col_move = {0:["L11","L21","L31"],1:["R12","L12","L22"],2:["R13","R23","L13"],3:["R14","R24","R34"]}
row_move = {0:["U11","U21","U31"],1:["D12","U12","U22"],2:["D13","D23","U13"],3:["D14","D24","D34"]}

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
    return (s == np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,0]])).all()

def solve(initial_board):
    global puzzle_tracking
    fringe = [initial_board]
    while len(fringe) > 0:
        s = fringe.pop(0)
        for s_prime, move in successor(s):
            if to_str(s_prime) in puzzle_tracking:
                if len(move) < len(puzzle_tracking[to_str(s_prime)]):
                    puzzle_tracking[to_str(s_prime)] = move
                continue
            else:
                puzzle_tracking[to_str(s_prime)] = puzzle_tracking[to_str(s)] + [move]
            if is_goal(s_prime):
                return(puzzle_tracking[to_str(s_prime)])
            fringe.append(s_prime)
    return False

# This is to use each state as the key of the dictionary to keep track of the moves
def to_str(s):
    return ''.join(str(elem) for inner in s for elem in inner)

filename = sys.argv[1]
S0 = initial_state(filename)
puzzle_tracking = {to_str(S0): []}

print solve(S0)

