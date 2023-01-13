##
##
##

from __future__ import annotations

import abc
from typing import NewType, Any, Dict, List

from utils import create_instance

State = NewType('State', Any)

Action = NewType('Action', Any)


class Environment(abc.ABC):

    @staticmethod
    def build(cfg: Dict[str, Any]) -> Environment:
        name = str(cfg['type']).lower()
        module = '.'.join(Environment.__module__.split('.')[:-1])
        class_path = f'{module}.{name.lower()}.{name.title()}Environment'

        return create_instance(class_path, Environment, cfg)

    @abc.abstractmethod
    def start(self) -> State:
        pass

    @abc.abstractmethod
    def actions(self, state: State) -> List[Action]:
        pass

    @abc.abstractmethod
    def result(self, state: State, action: Action) -> State:
        pass

    @abc.abstractmethod
    def is_goal(self, state: State) -> bool:
        pass

    @abc.abstractmethod
    def heuristic(self, state: State) -> int:
        pass

    @abc.abstractmethod
    def cost(self, src: State, action: Action, dest: State) -> int:
        pass
