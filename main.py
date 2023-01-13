##
##
##

from argparse import ArgumentParser
from pathlib import Path

import matplotlib.pyplot as plt
import yaml

from src.environment import Environment
from src.search import SearchGraph, SearchMode


def init_arg_parser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument('--env', type=str, required=True)

    return parser


"""
def a_star(env: GraphEnvironment) -> List:
    frontier = PriorityQueue()
    explored = set()
    parent: Dict = {}

    goal = None
    start = env.start_state()
    frontier.put(start, env.heuristic(start))
    parent[start] = None

    while not frontier.empty():
        current, p = frontier.pop()
        cost = p - env.heuristic(current)

        if env.is_goal(current):
            goal = current

        explored.add(current)

        for child in env.neighbors(current):
            f = env.heuristic(child) + cost + env.cost(current, child)
            if child not in frontier and child not in explored:
                frontier.put(child, f)
                parent[child] = current
            elif child in frontier:
                prev_f = frontier.priority(child)
                if f < prev_f:
                    frontier.update(child, f)
                    parent[child] = current

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]

    path.reverse()
    return path


def a_star2(env: GraphEnvironment) -> List:
    frontier = PriorityQueue()
    cost_so_far: Dict[Any, int] = {}
    parent: Dict = {}

    goal = None
    start = env.start_state()
    frontier.put(start, env.heuristic(start))
    parent[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current, p = frontier.pop()

        if env.is_goal(current):
            goal = current

        for child in env.neighbors(current):
            child_cost = cost_so_far[current] + env.cost(current, child)
            if child not in cost_so_far or child_cost < cost_so_far[child]:
                cost_so_far[child] = child_cost
                parent[child] = current

                f = child_cost + env.heuristic(child)
                frontier.put(child, f)

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]

    path.reverse()
    return path
"""


def main():
    parser = init_arg_parser()
    args = parser.parse_args()

    cfg_path = Path(args.env)
    with cfg_path.open('r') as f:
        cfg = yaml.safe_load(f)

    env = Environment.build(cfg)
    sg = SearchGraph.bfs(env, SearchMode.GRAPH)

    fig, ax = plt.subplots(nrows=1, ncols=1)
    sg.draw(ax)
    plt.show()


if __name__ == '__main__':
    main()
