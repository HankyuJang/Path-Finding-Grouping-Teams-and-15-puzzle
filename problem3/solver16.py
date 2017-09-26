#!/usr/bin/env python
# 

import sys
import numpy as np

# pos_move = {0: (L11, L21, L31, U11, U21, U31),
            # 1: (R12, L12, L22, U11, U21, U31),
            # 2: (R12, L12, L22, U11, U21, U31),
            # 3: (R12, L12, L22, U11, U21, U31),
            # }

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
        successor_list.append(s_next)

    return successor_list
    

filename = sys.argv[1]
S0 = initial_state(filename)
Sprime = successor(S0)
print Sprime

