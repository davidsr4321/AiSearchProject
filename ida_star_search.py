# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

from abc import ABC, abstractmethod
from heapq import heappush, heappop
from typing import Iterable, Union, TypeVar, Generic

# infinity as a constant
Infinite = float("inf")

# introduce generic type
T = TypeVar("T")


class IDAStar(ABC, Generic[T]):
    __slots__ = ()
    number_of_expansion = 0
    number_of_generated = 0
    max_path_len = 0

    class SearchNode:
        """Representation of a search node"""

        __slots__ = ("data", "closed", "came_from")

        def __init__(
                self, data: T
        ) -> None:
            self.data = data
            self.closed = False
            self.came_from = None

    class SearchNodeDict(dict):
        def __missing__(self, k):
            v = IDAStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    @abstractmethod
    def heuristic_cost_estimate(self, current: T, goal: T) -> float:
        """
        Computes the estimated (rough) distance between a node and the goal.
        The second parameter is always the goal.
        This method must be implemented in a subclass.
        """
        raise NotImplementedError

    @abstractmethod
    def distance_between(self, n1: T, n2: T) -> float:
        """
        Gives the real distance between two adjacent nodes n1 and n2 (i.e n2
        belongs to the list of n1's neighbors).
        n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
        This method must be implemented in a subclass.
        """

    @abstractmethod
    def neighbors(self, node: T) -> Iterable[T]:
        """
        For a given node, returns (or yields) the list of its neighbors.
        This method must be implemented in a subclass.
        """
        raise NotImplementedError

    def is_goal_reached(self, current: T, goal: T) -> bool:
        """
        Returns true when we can consider that 'current' is the goal.
        The default implementation simply compares `current == goal`, but this
        method can be overwritten in a subclass to provide more refined checks.
        """
        if type(current) is tuple and type(goal) is tuple:
            return current[1] == goal[1]
        return current == goal

    def reconstruct_path(self, last: SearchNode, reversePath=False) -> Iterable[T]:
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def idastar(
            self, start: T, goal: T, reversePath: bool = False
    ) -> Union[Iterable[T], None]:
        searchNodes = IDAStar.SearchNodeDict()
        startNode = searchNodes[start] = IDAStar.SearchNode(start)
        threshold = self.heuristic_cost_estimate(start, goal)
        while True:
            # print("Iteration with threshold: " + str(threshold))
            found, distance, node = self.idastar_rec(startNode, goal, threshold, searchNodes, start,0)
            if distance == Infinite:
                # Node not found and no more nodes to visit
                return None
            elif found:
                # if we found the node, construct the path
                # print("Found the node we're looking for!")
                return self.reconstruct_path(node, reversePath)
            else:
                # if it hasn't found the node, it returns the (positive) next-bigger threshold
                threshold = distance

    def idastar_rec(
            self, current: SearchNode, goal: T, threshold, searchNodes, start, counter
    ):
        counter += 1

        if self.is_goal_reached(current.data, goal):
            # We have found the goal node we we're searching for
            if counter > self.max_path_len:
                self.max_path_len = counter
            return True, 0, current

        estimate = self.distance_between(start, current.data) + self.heuristic_cost_estimate(current.data, goal)
        if estimate > threshold:
            # print("Breached threshold with heuristic: " + str(estimate))
            if counter > self.max_path_len:
                self.max_path_len = counter
            return False, estimate, None

        # ...then, for all neighboring nodes....
        min_value = Infinite

        self.number_of_expansion += 1
        for neighbor in map(lambda n: searchNodes[n], self.neighbors(current.data)):
            neighbor.came_from = current
            self.number_of_generated += 1  # generate: insert to the open list but don't expand
            found, distance, node = self.idastar_rec(neighbor, goal, threshold, searchNodes, start, counter)
            if found:
                # Node found
                return True, 0, node
            elif distance < min_value:
                min_value = distance

        return False, min_value, None


def find_path(
        start: T,
        goal: T,
        neighbors_fnct,
        reversePath=False,
        heuristic_cost_estimate_fnct=lambda a, b: Infinite,
        distance_between_fnct=lambda a, b: 1.0,
        is_goal_reached_fnct=lambda a, b: a == b,
) -> Union[Iterable[T], None]:
    """A non-class version of the path finding algorithm"""

    class FindPath(IDAStar):
        def heuristic_cost_estimate(self, current, goal):
            return heuristic_cost_estimate_fnct(current, goal)

        def distance_between(self, n1, n2):
            return distance_between_fnct(n1, n2)

        def neighbors(self, node):
            return neighbors_fnct(node)

        def is_goal_reached(self, current, goal):
            return is_goal_reached_fnct(current, goal)

    return FindPath().idastar(start, goal, reversePath)


__all__ = ["IDAStar", "find_path"]
