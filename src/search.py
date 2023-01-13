##
##
##

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Final, Optional, List, Callable, Dict, Tuple

import igraph as ig
from matplotlib.pyplot import Axes

from src.environment import Environment, State, Action
from utils.priority_queue import PriorityQueue, PriorityItem


@dataclass(unsafe_hash=True)
class Node:
    state: State = field(compare=True)
    parent: Optional[Node] = field(compare=True)
    action: Optional[Action] = field(compare=False)
    path_cost: int = field(compare=False)
    counter: int = field(hash=False)
    explored: bool = field(compare=False)

    @staticmethod
    def create(env: Environment,
               parent: Optional[Node],
               action: Optional[Action],
               counter: int) -> Node:
        if parent is None:
            state = env.start()
            path_cost = 0
        elif action is None:
            raise ValueError(f'Action not defined.')
        else:
            state = env.result(parent.state, action)
            path_cost = parent.path_cost + env.cost(parent.state, action, state)

        return Node(state, parent, action, path_cost, counter, False)


class SearchMode(Enum):
    NAIVE_TREE = 'naive_tree'
    NO_LOOP_TREE = 'no_loop_tree'
    BEST_TREE = 'best_tree'
    REP_BEST_TREE = 'rep_best_tree'
    NAIVE_GRAPH = 'naive_graph'

    def tree(self) -> bool:
        return self == SearchMode.NAIVE_TREE or self == SearchMode.NO_LOOP_TREE


def find_loop(node: Node) -> bool:
    current = node.parent
    found = False
    while current is not None and not found:
        found = current.state == node.state
        current = current.parent

    return found


class SearchGraph:
    _nodes: Final[List[Node]]

    def __init__(self):
        self._nodes = []

    def _add_node(self, node: Node):
        self._nodes.append(node)

    def draw(self, ax: Axes):
        nodes_idx = {}
        for idx, node in enumerate(self._nodes):
            nodes_idx[node] = idx

        edges = []
        for node in self._nodes:
            if node.parent is not None:
                edges.append((nodes_idx[node.parent], nodes_idx[node]))

        graph = ig.Graph(len(self._nodes), edges)
        ig.plot(graph, target=ax, layout='circle')

    @staticmethod
    def best_first_search(env: Environment,
                          mode: SearchMode,
                          f: Callable[[Environment, Node], int],
                          cmp: Callable[[Tuple[Node, int], Tuple[Node, int]], bool]):
        reached: Dict[State, PriorityItem[Node, int]] = {}
        frontier = PriorityQueue(
            lambda x, y: cmp((x.value, x.priority), (y.value, y.priority)))
        counter = 0

        start = Node.create(env, None, None, counter)
        counter += 1
        tmp = frontier.put(start, f(env, start))

        if not mode.tree():
        if not mode.tree():
            reached[start.state] = tmp

        while not frontier.empty():
            node, priority = frontier.pop()
            node.explored = True

            if env.is_goal(node.state):
                break

            for action in env.actions(node.state):
                child = Node.create(env, node, action, counter)

                match mode:
                    case SearchMode.NAIVE_TREE:
                        frontier.put(child, f(env, child))
                        counter += 1
                    case SearchMode.NO_LOOP_TREE:
                        if not find_loop(child):
                            frontier.put(child, f(env, child))
                            counter += 1
                    case SearchMode.BEST_TREE:
                        if child.state not in reached or f(env, child) < reached[child.state].priority:
                            reached[child.state] = frontier.put(child, f(env, child))
                            counter += 1
                    case SearchMode.REP_BEST_TREE:
                        if child.state not in reached or reached[child.state].value.explored:
                            reached[child.state] = frontier.put(child, f(env, child))
                            counter += 1
                        elif not reached[child.state].value.explored:
                            reached[child.state] = frontier.update(reached[child.state], child, f(env, child))
                            counter += 1
                    case SearchMode.NAIVE_GRAPH:
                        if child.state not in reached:
                            reached[child.state] = frontier.put(child, f(env, child))
                            counter += 1
                        elif not reached[child.state].value.explored and f(env, child) < reached[child.state].priority:
                            reached[child.state] = frontier.update(reached[child.state], child, f(env, child))
                            counter += 1

    """
    @staticmethod
    def bfs(env: Environment, mode: SearchMode) -> SearchGraph:
        graph = SearchGraph()

        frontier: Queue[Node] = Queue()
        frontier_set: Set[State] = set()
        explored: Set[State] = set()

        start = Node(env.start(), None, 0)
        graph._add_node(start)
        if env.is_goal(start.state):
            return graph
        frontier.put(start)

        while not frontier.empty():
            current = frontier.get()
            explored.add(current.state)

            for child_state in env.neighbors(current.state):
                child_node = Node(child_state, current, current.cost + 1)
                graph._add_node(child_node)

                if child_state not in explored and child_state not in frontier_set:
                    if env.is_goal(child_state):
                        return graph
                    frontier.put(child_node)

        return graph
    """

    @staticmethod
    def depth_first_search(env: Environment, mode: SearchMode):
        pass

    @staticmethod
    def astar(env: Environment, mode: SearchMode):
        pass
