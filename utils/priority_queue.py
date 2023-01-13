##
##
##

from dataclasses import dataclass, astuple

from typing import List, TypeVar, Generic, Callable, Optional, Dict

T = TypeVar('T')
P = TypeVar('P', bound=int)


@dataclass(init=True, frozen=True, eq=True)
class PriorityItem(Generic[T, P]):
    value: T
    priority: P

    def __iter__(self):
        return iter(astuple(self))


def parent(i: int) -> int:
    return (i - 1) // 2


def left(i: int) -> int:
    return 2 * i + 1


def right(i: int) -> int:
    return 2 * i + 2


# Highly inefficient implementation of a priority queue
class PriorityQueue(Generic[T, P]):
    _items: List[PriorityItem[T, P]]
    _pos: Dict[PriorityItem, int]
    _cmp: Callable[[PriorityItem[T, P], PriorityItem[T, P]], bool]

    def __init__(self, cmp: Callable[[PriorityItem[T, P], PriorityItem[T, P]], bool]):
        self._items = []
        self._pos = {}
        self._cmp = cmp

    def _swap(self, i: int, j: int):
        tmp = self._items[i]
        self._items[i] = self._items[j]
        self._items[j] = tmp

        self._pos[self._items[i]] = i
        self._pos[self._items[j]] = j

    def _heap_restore(self, i: int, dim: int):
        min_pos = i
        if left(i) <= dim and self._cmp(self._items[left(i)], self._items[min_pos]):
            min_pos = left(i)
        if right(i) <= dim and self._cmp(self._items[right(i)], self._items[min_pos]):
            min_pos = right(i)

        if i != min_pos:
            self._swap(i, min_pos)
            self._heap_restore(min_pos, dim)

    def empty(self) -> bool:
        return len(self._items) == 0

    def put(self, value: T, priority: P) -> PriorityItem[T, P]:
        tmp = PriorityItem(value, priority)
        self._items.append(tmp)
        self._pos[tmp] = len(self._items) - 1

        i = len(self._items) - 1
        while i > 0 and self._cmp(self._items[i], self._items[parent(i)]):
            self._swap(i, parent(i))
            i = parent(i)

        return tmp

    def top(self) -> Optional[PriorityItem[T, P]]:
        if self.empty():
            return None
        return self._items[0]

    def pop(self) -> Optional[PriorityItem[T, P]]:
        if self.empty():
            return None

        self._swap(0, len(self._items) - 1)
        self._heap_restore(0, len(self._items) - 2)
        return self._items.pop()

    def update(self, item: PriorityItem, value: Optional[T] = None, priority: Optional[P] = None) -> PriorityItem[T, P]:
        new_item = item

        if value is not None:
            new_item = PriorityItem(value, item.priority)
            i = self._pos.pop(item)
            self._items[i] = new_item
            self._pos[new_item] = i
            item = new_item

        if priority is not None:
            new_item = PriorityItem(item.value, priority)
            if not self._cmp(new_item, item):
                raise ValueError(f'New priority item does not come before the old one.')

            i = self._pos.pop(item)
            self._pos[new_item] = i
            while i > 0 and self._cmp(self._items[i], self._items[parent(i)]):
                self._swap(i, parent(i))
                i = parent(i)

        return new_item
