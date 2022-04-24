def calc_target_bearing(robot):
    if int(robot.zone) == 0:
        return 105
    elif int(robot.zone) == 1:
        return 195
    elif int(robot.zone) == 2:
        return 285
    elif int(robot.zone) == 3:
        return 15