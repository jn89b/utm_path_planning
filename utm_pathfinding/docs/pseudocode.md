2 nodes connected create an edge

```python
class AbstractNode():
    def __init__(self, position:tuple) -> None:
        self.position = position

    def __str__(self) -> str:
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def add_neighbor(self, neighbor:AbstractNode, weight=0) -> None:
        self.adjacent[neighbor] = weight

    def get_connections(self) -> dict.keys():
        return self.adjacent.keys()  

    def get_position(self) -> tuple:
        return self.position

    def get_weight(self, neighbor:AbstractNode):
        return self.adjacent[neighbor]

class AbstractGraph():
    def __init__(self, map_area:Map) -> None:
        self.map_area = map_area
        self.entrances = {}

    def abstract_graph(self):
        """pass"""
        regions = self.map_area.regions
        for k, reg in regions.items():
            for (reg_side, nei_side) in zip()

    def build_inter_connections(self):
        for e
        

```