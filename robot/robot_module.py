class RobotModule:

    def __init__(self, name, description, version, minimum_robot_version,
                 maximum_robot_version):
        self.name = name
        self.description = description
        self.version = version
        self.minimum_robot_version = minimum_robot_version
        self.maximum_robot_version = maximum_robot_version

    def __str__(self):
        return "RobotModule(name={}, description={}, version={}, minimum_robot_version={}, maximum_robot_version={})".format(
            self.name, self.description, self.version,
            self.minimum_robot_version, self.maximum_robot_version)
