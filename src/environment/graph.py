##
##
##

import warnings
from typing import Dict, Any, List, Final

import networkx as nx

from .base import Environment, State, Action


class GraphEnvironment(Environment):
    _graph: Final[nx.Graph]
    _start: State

    def __init__(self, cfg: Dict[str, Any]):
        self._graph = nx.Graph()

        states = cfg['states']
        directed = False
        if 'directed' in cfg:
            directed = bool(cfg['directed'])

        self._add_nodes(states)
        self._add_edges(states, directed)

    def _add_nodes(self, states: List[Dict[str, Any]]):
        start_state = None
        goal_assigned = False

        for state in states:
            label = state['label']
            if self._graph.has_node(label):
                raise ValueError(f'Label \'{label}\' assigned to more than one state.')

            start = False
            if 'start' in state:
                start = bool(state['start'])

                if start and start_state is not None:
                    raise ValueError('Only one state can be the start state.')
                elif start:
                    start_state = label

            goal = False
            if 'goal' in state:
                goal = bool(state['goal'])
                goal_assigned = goal_assigned or goal

            if goal:
                heuristic = 0
                if 'heuristic' in state:
                    warnings.warn(f'Heuristic for goal states is ignored.')
            elif 'heuristic' in state:
                heuristic = int(state['heuristic'])
            else:
                raise ValueError(f'Missing heuristic for state \'{label}\'.')

            self._graph.add_node(label, heuristic=heuristic, start=start, goal=goal)

        if start_state is None:
            raise ValueError('One state must be the start state.')
        if not goal_assigned:
            raise ValueError('At least one state must be a goal.')

        self._start = start_state

    def _add_edges(self, states: List[Dict[str, Any]], directed: bool):
        for state in states:
            if 'neighbors' not in state:
                continue

            f = state['label']
            for neighbor in state['neighbors']:
                t = neighbor['label']
                cost = int(neighbor['cost'])

                if not self._graph.has_node(t):
                    raise ValueError(f'No state with label \'{f}\' was defined.')

                if self._graph.has_edge(f, t) \
                        or (self._graph.has_edge(t, f) and not directed) \
                        and self._graph.edges[f, t]['cost'] != cost:
                    raise ValueError(f'Edge \'{f}\'-\'{t}\' was declared \
                                    twice with different costs.')

                self._graph.add_edge(f, t, cost=cost)
                if not directed:
                    self._graph.add_edge(t, f, cost=cost)

    def start(self) -> State:
        return self._start

    def actions(self, state: State) -> List[Action]:
        return list(range(len(self._graph[state])))

    def result(self, state: State, action: Action) -> State:
        return list(self._graph[state])[action]

    def is_goal(self, state: State) -> bool:
        return self._graph.nodes[state]['goal']

    def heuristic(self, state: State) -> int:
        return self._graph.nodes[state]['heuristic']

    def cost(self, src: State, action: Action, dest: State) -> int:
        return self._graph.edges[src, dest]['cost']
