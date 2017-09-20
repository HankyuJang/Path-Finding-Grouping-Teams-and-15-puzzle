#!/usr/bin/env python
# 
# assign.py : Solve the student grouping problem
# (1)   
# State space: Each team is represented in a list, a list containing the total teal, and a outer list containing all the possible formulation of teams.
# Successor function: Join two teams (Reduce one team by grouping the two teams)
# 
# S3 = [['kapadia'], ['chen464'], ['steflee', 'fan6'], ['zehzhang', 'djcran']]
# S4 = successor(S3)
#
# then S4 is
# [[['steflee', 'fan6'], ['zehzhang', 'djcran'], ['kapadia', 'chen464']], 
# [['chen464'], ['zehzhang', 'djcran'], ['kapadia', 'steflee', 'fan6']], 
# [['chen464'], ['steflee', 'fan6'], ['kapadia', 'zehzhang', 'djcran']], 
# [['kapadia'], ['zehzhang', 'djcran'], ['chen464', 'steflee', 'fan6']], 
# [['kapadia'], ['steflee', 'fan6'], ['chen464', 'zehzhang', 'djcran']]]
#
# Edge weights: 0 (I didn't consider the time in formulating team in this problem)
# Goal state: A state with the minimum cost
# Heuristic function: X
#
# (2) How the search algorithm work
#
# For each step the algorithm branches, and then calculates the cost of each child node. If the cost is greater than it's parent, it doesn't push that child node in the fringe. As the algorithm proceed, it will search through the whole space if the child node is guaranteed to give a lower cost than its parent.
#
# (3) Any problem I faced, assumptions, simplifications, design decisions
#
# One assumption: I assumed that if the child node has higher cost, then in its subtree, it cannot have a team grouping that has lower cost than its parent.


import sys
import copy

def readfile(filename):
    student_dict = {}
    file = open(filename, "r")
    for line in file:
        line = line.split()
        group_size_pref = int(line[1])
        if line[2] == '_':
            to_work_pref = []
        else:
            to_work_pref = line[2].split(',')
        if line[3] == '_':
            not_to_work_pref = []
        else:
            not_to_work_pref = line[3].split(',')
        student_dict[line[0]] = tuple([group_size_pref, to_work_pref, not_to_work_pref])
    return student_dict

def initial_state(student_dict):
    student_list = student_dict.keys()
    return [[student] for student in student_list]

def successor(s):
    successor_list = []
    for i, student_i in enumerate(s):
        for j, student_j in enumerate(s[i+1:]):
            new_pair = student_i + student_j
            if len(new_pair) > 3: # do not form a team > 3
                continue
            new_group = copy.deepcopy(s)
            new_group.remove(student_i)
            new_group.remove(student_j)
            new_group += [new_pair]
            # Sort the team
            [team.sort() for team in new_group]
            new_group.sort()
            successor_list.append(new_group)
    return successor_list
            
def calculate_cost(s):
    comp1 = k * len(s)
    comp2 = 0
    comp3 = 0
    comp4 = 0
    for team in s: # for each team in s
        n_members = len(team)
        for student in team:
            group_size_pref = student_dict[student][0]
            if group_size_pref != 0 and group_size_pref != n_members:
                comp2 += 1
            
            to_work_pref = student_dict[student][1]
            for member in to_work_pref:
                if member not in team:
                    comp3 += n

            not_to_work_pref = student_dict[student][2]
            for member in not_to_work_pref:
                if member in team:
                    comp4 += m

    return comp1 + comp2 + comp3 + comp4

def solve(initial_state):
    global groups_tracking
    fringe = [initial_state]
    s_best = initial_state
    min_cost = calculate_cost(initial_state)
    while len(fringe) > 0:
        s = fringe.pop()
        s_cost = calculate_cost(s)
        for s_prime in successor(s):
            # Check if successor is already considered previously
            if s_prime in groups_tracking:
                continue
            else:
                groups_tracking += [s_prime]
            
            s_prime_cost = calculate_cost(s_prime)
            if s_prime_cost < s_cost:
                fringe.append(s_prime)
            if s_prime_cost < min_cost:
                min_cost = s_prime_cost
                s_best = s_prime
    return s_best, min_cost

# def solve_bf(initial_state):
    # global groups_tracking
    # fringe = [initial_state]
    # s_best = initial_state
    # min_cost = calculate_cost(initial_state)
    # while len(fringe) > 0:
        # s = fringe.pop()
        # s_cost = calculate_cost(s)
        # for s_prime in successor(s):
            # # Check if successor is already considered previously
            # if s_prime in groups_tracking:
                # continue
            # else:
                # groups_tracking += [s_prime]

            # s_prime_cost = calculate_cost(s_prime)
            # fringe.append(s_prime)
            # if s_prime_cost < min_cost:
                # min_cost = s_prime_cost
                # s_best = s_prime
    # return s_best, min_cost

def printable_result(s):
    return "\n".join([" ".join([member for member in team]) for team in s])

filename = sys.argv[1]
k = int(sys.argv[2])
m = int(sys.argv[3])
n = int(sys.argv[4])
student_dict = readfile(filename)

S0 = initial_state(student_dict)
S0.sort()
groups_tracking = [S0]

s_best, min_cost = solve(S0)
print printable_result(s_best)
print min_cost

# print("-"*40)
# print("Now checking using the bruteforce way\n")
# s_best, min_cost = solve_bf(S0)
# print printable_result(s_best)
# print min_cost

# Checking the calculate_cost function with the example in the problem2
# s_goal = [['djcran', 'chen464'], ['kapadia', 'zehzhang', 'fan6'], ['steflee']]
# print calculate_cost(s_goal)

