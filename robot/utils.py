def calc_target_bearing(robot):
    if int(robot.zone) == 0:
        return 105
    elif int(robot.zone) == 1:
        return 195
    elif int(robot.zone) == 2:
        return 285
    elif int(robot.zone) == 3:
        return 15
def bearing_of_zero_for_zones(zone):
    route = 2
    if int(zone) == 0:
        return 90+87
    elif int(zone) == 1:
        return 180+87
    elif int(zone) == 2:
        return 270+87
    elif int(zone) == 3:
        return 0+87