import math
import sys
import time
import math
import random

import numpy as np
import unittest

from a_star_search import AStar
from ida_star_search import IDAStar


class RobberSolver(AStar):

    def __init__(self, money):
        self.money = money

    def heuristic_cost_estimate(self, n1, n2):
        (_, index1) = n1
        (_, index2) = n2
        # in order to estimate the maximal values to be chosen currently
        k_values_to_pick = math.ceil((index2 - index1) / 2)
        # get the indices of the top k_values_to_pick in the money list in the range of [index1, index2] and sum it
        # all together
        return sum(-m for m in np.sort(self.money[index1:index2])[-k_values_to_pick:])

    def distance_between(self, n1, n2):
        (money1, _) = n1
        (money2, _) = n2
        return money2 - money1

    def neighbors(self, node):
        curr_money, index = node
        if index == len(self.money):
            return []
        elif index == len(self.money) - 1:  # if we are at the end, we can only take this one time
            return [(curr_money - self.money[index], index + 1)]
        else:  # if we are at the end, we can only take this one time
            return [(curr_money, index + 1), (curr_money - self.money[index], index + 2)]
        # we can either not take this current one or take and not take the next one

    def is_goal_reached(self, current, goal):
        if type(current) is tuple and type(goal) is tuple:
            return current[1] == goal[1]
        return current == goal


class RobberSolverUCS(AStar):
    def __init__(self, money):
        self.money = money

    def heuristic_cost_estimate(self, n1, n2):
        return 0

    def distance_between(self, n1, n2):
        (money1, _) = n1
        (money2, _) = n2
        return money2 - money1

    def neighbors(self, node):
        curr_money, index = node
        if index == len(self.money):
            return []
        elif index == len(self.money) - 1:  # if we are at the end, we can only take this one time
            return [(curr_money - self.money[index], index + 1)]
        else:  # if we are at the end, we can only take this one time
            return [(curr_money, index + 1), (curr_money - self.money[index], index + 2)]
        # we can either not take this current one or take and not take the next one

    def is_goal_reached(self, current, goal):
        return current == goal


class RobberSolver_IDAstar(IDAStar):
    def __init__(self, money):
        self.money = money

    def heuristic_cost_estimate(self, n1, n2):
        (_, index1) = n1
        (_, index2) = n2
        # in order to estimate the maximal values to be chosen currently
        k_values_to_pick = math.ceil((index2 - index1) / 2)
        # get the indices of the top k_values_to_pick in the money list in the range of [index1, index2] and sum it
        # all together
        return sum(-m for m in np.sort(self.money[index1:index2])[-k_values_to_pick:])

    def distance_between(self, n1, n2):
        (money1, _) = n1
        (money2, _) = n2
        return money2 - money1

    def neighbors(self, node):
        curr_money, index = node
        if index == len(self.money):
            return []
        elif index == len(self.money) - 1:  # if we are at the end, we can only take this one time
            return [(curr_money - self.money[index], index + 1)]
        else:  # if we are at the end, we can only take this one time
            return [(curr_money, index + 1), (curr_money - self.money[index], index + 2)]
        # we can either not take this current one or take and not take the next one


def rob(nums):
    rob1, rob2 = 0, 0
    for n in nums:
        temp = max(rob1 + n, rob2)
        rob1 = rob2
        rob2 = temp
    return rob2


def plan_robberyAStar(money, optimal):
    start = (0, 0)  # we begin with house 0 and 0 money
    goal = (0, len(money))  # the goal node isn't really important, the stopping condition is depended only in the index

    robber = RobberSolver(money)
    startT = time.time()
    path = list(robber.astar(start, goal))
    endT = time.time()
    a = path[len(path) - 1][0]
    # print("A*: ")
    # print("optimal cost by search: " + str(path[len(path) - 1][0]))
    # print("  real cost by dynamic: " + str(-rob(money)))
    # print(path)
    # print("number of node expanded: " + str(robber.number_of_expansion))
    # print("number of node generated: " + str(robber.number_of_generated))
    # print("totalTime: " + str(endT-startT))
    # print("space complexity, number of nodes: " + str(len(robber.openSet)+robber.number_of_closed))
    # print("ok\n")
    return robber.number_of_expansion, robber.number_of_generated, len(
        robber.openSet) + robber.number_of_closed, endT - startT


def plan_robberyUCS(money, optimal):
    start = (0, 0)  # we begin with house 0 and 0 money
    goal = (-rob(money),
            len(money))  # the goal node isn't really important, the stopping condition is depended only in the index

    robber = RobberSolverUCS(money)

    startT = time.time()
    path = list(robber.astar(start, goal))
    endT = time.time()
    a = path[len(path) - 1][0]
    # print("UCS: ")

    # print("optimal cost by search: " + str(path[len(path) - 1][0]))
    # print("  real cost by dynamic: " + str(-rob(money)))
    # print(path)
    # print("number of node expanded: " + str(robber.number_of_expansion))
    # print("number of node generated: " + str(robber.number_of_generated))
    # print("space complexity, number of nodes: " + str(len(robber.openSet) + robber.number_of_closed))
    # print("totalTime: " + str(endT-startT))
    # print("ok\n")
    return robber.number_of_expansion, robber.number_of_generated, len(
            robber.openSet) + robber.number_of_closed, endT - startT


# n_expanded1, n_generated1, space1, time1

def plan_robberyIDA(money, optimal):
    start = (0, 0)  # we begin with house 0 and 0 money
    goal = (-rob(money),
            len(money))  # the goal node isn't really important, the stopping condition is depended only in the index

    robber = RobberSolver_IDAstar(money)

    startT = time.time()
    path = list(robber.idastar(start, goal))
    endT = time.time()

    a = path[len(path) - 1][0]
    # print("IDA*: ")

    # print("optimal cost by search: " + str(path[len(path) - 1][0]))
    # print("  real cost by dynamic: " + str(-rob(money)))
    # print(path)
    # print("number of node expanded: " + str(robber.number_of_expansion))
    # print("number of node generated: " + str(robber.number_of_generated))
    # print("space complexity: " + str(robber.max_path_len))
    # print("totalTime: " + str(endT - startT))
    # print("ok\n\n")
    # n_expanded1, n_generated1, space1, time1
    return robber.number_of_expansion, robber.number_of_generated, robber.max_path_len, endT - startT
    # n_expanded, n_generated, space, time


if __name__ == '__main__':
    # check = [60,50,70,100,20]
    # a = [math.ceil((len(check) - i) / 2) for i in range(len(check))]
    # print (check)
    # print(a)

    # plan_robbery(check)

    # Test case 1:
    # for i in range(100):
    lengths_to_check = set([0,5,10,15,20,25,30,35,40,45,50,55,60])
    for house_length in lengths_to_check:
        # A*:
        n_expanded_a_star_total = 0
        n_generated_a_star_total = 0
        space_a_star_total = 0
        time_a_star_total = 0

        # UCS
        n_expanded_UCS_total = 0
        n_generated_UCS_total = 0
        space_UCS_total = 0
        time_UCS_total = 0

        # IDA*:
        n_expanded_ida_star_total = 0
        n_generated_ida_star_total = 0
        space_ida_star_total = 0
        time_ida_star_total = 0

        number_of_checks = 1
        for i in range(number_of_checks):
            random_money = [random.randint(1, 100) for j in range(house_length)]
            optimal = -rob(random_money)

            n_expanded1, n_generated1, space1, time1 = plan_robberyAStar(random_money, optimal)
            n_expanded_a_star_total += n_expanded1
            n_generated_a_star_total += n_generated1
            space_a_star_total += space1
            time_a_star_total += time1

            n_expanded2, n_generated2, space2, time2 = plan_robberyUCS(random_money, optimal)
            n_expanded_UCS_total += n_expanded2
            n_generated_UCS_total += n_generated2
            space_UCS_total += space2
            time_UCS_total += time2

            n_expanded3, n_generated3, space3, time3 = plan_robberyIDA(random_money, optimal)
            n_expanded_ida_star_total += n_expanded3
            n_generated_ida_star_total += n_generated3
            space_ida_star_total += space3
            time_ida_star_total += time3

        print("------------------- for " + str(house_length) + " houses:---------------------- ")
        print("A*:")
        print("number of node expanded: " + str(n_expanded_a_star_total / number_of_checks))
        print("number of node generated: " + str(n_generated_a_star_total / number_of_checks))
        print("space complexity: " + str(space_a_star_total / number_of_checks))
        print("totalTime: " + str(time_a_star_total / number_of_checks))
        print("\n")
        print("UCS:")
        print("number of node expanded: " + str(n_expanded_UCS_total / number_of_checks))
        print("number of node generated: " + str(n_generated_UCS_total / number_of_checks))
        print("space complexity: " + str(space_UCS_total / number_of_checks))
        print("totalTime: " + str(time_UCS_total / number_of_checks))
        print("\n")
        print("IDA*:")
        print("number of node expanded: " + str(n_expanded_ida_star_total / number_of_checks))
        print("number of node generated: " + str(n_generated_ida_star_total / number_of_checks))
        print("space complexity: " + str(space_ida_star_total / number_of_checks))
        print("totalTime: " + str(time_ida_star_total / number_of_checks))
        print("------------------- end of " + str(house_length) + " houses:---------------------- ")
        print("\n\n\n")
    # index2 = len(money)
    # for i in range(len(money)):
    #     # in order to estimate the maximal values to be chosen currently
    #     index1 = i
    #     k_values_to_pick = math.ceil((index2 - index1) / 2)
    #     get the indices of the top k_values_to_pick in the money list in the range of [index1, index2] and sum it all
    #     # together
    #     print(sum(-m for m in np.sort(money[index1:index2])[-k_values_to_pick:]))
