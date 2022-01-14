# TODO: reconsider how we do all this
# SLAM based a* pathfinding would be awesome but we would realy need a lidar to do that

from robot_module import RobotModule

class Pathfinder(RobotModule):
    def __init__(self, mapper_instance):
        super().__init__("pathfinder", "Pathfinder system", "0.0.1", "0.0.0", "1.0.0")
        self.mapper_instance = mapper_instance
        self.goal = None
        self.path = None
        self.path_index = 0
        self.path_length = 0
        self.path_found = False
        self.reached_goal = False
    
    def set_mapper(self, mapper_instance):
        self.mapper_instance = mapper_instance
    
    def set_goal(self, goal):
        self.goal = goal
        self.calculate_path()
    
    def calculate_path(self):
        # well this is going to be fun to implement :|
        
        # get all the obstacles, between us and the goal
        obstacles = [] #self.mapper_instance.get_obstacles_between(self.goal)
        # add a "VirtualNode" to the map for the goal and start point
        #self.mapper_instance.add_node()
        # Calculate untraversable map segments
        # Calculate traversable map segments
        # Calculate path using A*
        # Add the path to the map, and set the goal to the last node in the path
        # Set the path_found flag to True
        # Set the path_index to 0
        # Set the path_length to the length of the path
        # Set the reached_goal flag to False
        # Tell the movement module to start moving towards the nearst node in the path

        pass
