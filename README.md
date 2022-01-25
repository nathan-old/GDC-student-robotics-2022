# Goldalming college student robotics
Repo for Godalming College Student Robotics and all things orange.

# Deploying
First run the unit tests with:
```
python3 scripts/run_not_yet_made_unit_tests
```
If they fail, run and cry

To produce the depoyable zip run:
```
python3 deploy.py
```

This will output to target/robot.zip

# Running
After powering on the robot, wait for either of the preboot audio status codes described bellow and then press the start button. The robot will try to boot, launch modules and perform diagnostic checks and will spit out the appropriate audio code for failures, hopefully. Please check logs if you are unsure and spam leo on discord if its broken because its probably his fault.

# Debuging using buzzers
Please note, that without `VERBOSE_AUDIO = True` many of these will not play. 
The diffrent status codes:
- `C6 for 0.25, C6 for 0.2, silence for 0.25, C7 for 1`: Preboot, compition mode (you are probably doing something wrong unless you REALLY are in comp mode)
- `C6 for 0.25, C6 for 0.2, silence for 0.25, C6 for 1`: Preboot, dev mode (normal)
- `C6 for 0.5, silence for 0.25, C6 for 0.5, silence for 0.25 C6 for 0.5`: Robot hardware online sucsessfully 
- `E6 for 0.5, silence for 0.25, E6 for 0.5, silence for 0.25 E6 for 0.5, silence for 1`: Robot hardware failed to come online due to error; check logs
- `G6 for 0.1, E6 for 0.1, G6 for 0.1, G6 for 0.25, E6 for 0.25, silence for 1` (x5): Custom robot software failed to come online due to errors; check logs 
- `G6 for 0.1` (x2): Starting diagnostics (normal behaviour)
- `Decrease from high pitch (4khz) to low pitch (1khz)`: Diagnoistic failure, check logs
- `Increase from low pitch (1khz) to high pitch (4khz)`: Diagnoistic tests passed
- `C6 for 0.1, C6 for 0.1, B6 for 0.1, B6 for 0.1, G6 for 0.25, G6 for 0.25, silence for 1` (x5): Main loop failure, this indicates a problem with either your instructions.txt or idle code loop; check logs

# Folders
- 3d parts: Will contain cad designs relivant to the robot
- arduino: Contains the scripts for the arduino, apparently
- demos: Old scripts that were in this repo before i cleaned it up
- Robot: the code that will be deploied
# TODO
- Offset for camera to robot centre
- Offset to grabber
- Get camera library working
- Create mapping
- Include can locations
- Get positional mapping working with distance tracking
- Build can grabber
- Build lifting arm
- Code to use grabber
- Calibrate as distance is to centre of marker
- Debug display
- Battery enclosure
- Flag mount
- Cable management
- Cut down mounting bolts
- Notch mounting plates
- Create spare part assemblies
- Start switch/ button
- Hardcoded movement instructions
- Create return to home function
- Motor hubs/ grub screws
- Make USB's accessable for COMP mode + code USB stick
- Improve looks of robot
- Give robot a name
- Add GDC Branding
- RPM for different powers
- ~~Mark motor letters~~
- ~~Maths for positional data~~