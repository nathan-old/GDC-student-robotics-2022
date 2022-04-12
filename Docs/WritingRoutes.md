#Writing routes
The robot can run off of a text file (route.txt) which is found in the robot.zip file. It is made up off the following commands.
# Forwards
```forwards, DISTANCE, SIDE``` 
- DISTANCE: the (float) distance you want the robot to go in metres. Negative for reverse.
- SIDE: the side in which you want to be at the front (```Plough```, ```Empty''') grabber is default (in an else statement) so anything works
# Turn
```turn, ANGLE```
- ANGLE: the (float) angle you wish to turn in degrese. Negative for reverse.
# Sideways
```sideways, DISTANCE```
- DISTANCE: the (float) distance you want the robot to go in metres. Negative for reverse.
# Sleep
```sleep, TIME```
- TIME: the (float) time you wish the robot to sleep for.
# Grab
``` Grab, 0```
- needs a comma seperation to pass
#Beep
```beep, TIME```
- TIME: the (float) time you wish the robot to beep for.



#Comments
```\\, COMMAND```
- COMMAND: the infomation you wish to comment out/ ignore
