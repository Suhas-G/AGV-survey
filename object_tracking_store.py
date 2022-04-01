import numpy as np
from collections import deque
from pprint import pprint

DISTANCE_THRESHOLD = 0.5


class ObjectTrackingStore:
    def __init__(self) -> None:
        self.store = {}
        self.next_object_id = 0


    def push(self, object_id, position):
        if object_id not in self.store:
            self.store[object_id] = {
                'positions': deque(maxlen=10), 
                'mean': np.array([0, 0, 0]),
                'count': 0
            }

        length = len(self.store[object_id]['positions'])
        self.store[object_id]['mean'] = ((self.store[object_id]['mean'] * length) + position) / (length + 1)
        self.store[object_id]['positions'].append(position)
        self.store[object_id]['count'] = (self.store[object_id]['count'] + 1) % 10


        return self.store[object_id]['count']


    def query(self, position):
        distances = {
            object_id: np.linalg.norm(position - self.store[object_id]['mean'])
            for object_id in self.store
        }
        object_id = min(distances, key=distances.get)
        return object_id, distances[object_id]

    def query_and_push(self, position):
        if len(self.store) == 0:
            object_id = self.next_object_id
            count = self.push(object_id, position)
            self.next_object_id += 1
        else:
            object_id, distance = self.query(position)
            if distance <= DISTANCE_THRESHOLD:
                count = self.push(object_id, position)
            else:
                object_id = self.next_object_id
                count = self.push(object_id, position)
                self.next_object_id += 1

        return object_id, count



def _test():
    store = ObjectTrackingStore()
    store.query_and_push(np.array([0, 0, 0]))
    store.query_and_push(np.array([0, 0, 1]))
    store.query_and_push(np.array([1, 1, 1]))
    store.query_and_push(np.array([1, 1, 2]))
    store.query_and_push(np.array([2, 2, 2]))
    store.query_and_push(np.array([2, 2, 2.5]))
    store.query_and_push(np.array([0.5, 1, 1]))

    pprint(store.store)


if __name__ == '__main__':
    _test()