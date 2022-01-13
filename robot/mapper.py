from robot_module import RobotModule

class MapNode:
    def __init__(self, x, y, z, meta, virtual=False):
        self.x = x
        self.y = y
        self.z = z
        self.meta = meta
        self.virtual = virtual
    
    def __str__(self):
        return "MapNode(x={}, y={}, z={}, meta={}, virtual={})".format(self.x, self.y, self.z, self.meta, self.virtual)
    
class MapMananger(RobotModule): 
    def __init__(self, map_width, map_length, static_node_data =[], node_map_resolution=1):
        self.map_width = map_width
        self.map_length = map_length
        self.static_node_data = static_node_data
        self.node_map = []
        if len(static_node_data) / node_map_resolution != map_width or len(static_node_data[0]) / node_map_resolution != map_length:
            raise Exception("Static line data does not match map dimensions or line resolution")
        
        if (self.static_node_data != []): # A side note, if you pass static line data: ensure 
            self.node_map = self.static_node_data
        else:
            self.node_map =  [[MapNode(x, y, None, True) for x in range(map_length * node_map_resolution)] for y in range(map_width * node_map_resolution)]
        
        self.current_position_x = 0
        self.current_position_y = 0

    
        
    

        