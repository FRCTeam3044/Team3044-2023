![Charged-Up](charged-up.png)
## Features
### H-Drive Simulation
Creating a model of an H-Drive from either sysid characterization values or from robot information (motor type, num of motors, weight, gearbox ratio, etc). H-Drive creation and simulation are tucked away in a package for organization and convenience in porting over to other projects & are built to interface with the rest of WPIlibs features and packages.
### H-Drive Class
Creating an H-Drive from 3 motor groups with the base tank drive, and arcade drive classes implemented in a style following WPIlibs differential & mecanum drive systems including motor safety functions.
### Field Relative Drive
Both rotation-independent field relative drive & rotation-assisted field relative drive as H-Drive is not as effective moving laterally as it is moving linearly.
### April Tags
April tag simulation & detection.
- Simulating april tags in video feeds
- Simulation-based april tag detection for testing
- Detection on coprocessors & sent over NT4 system (To Be Tested)
### Custom Kalman Filters
Kalman filter for H-Drive based odometry & kinematics.
- Encoder readouts from all drive wheels
- Gyro input
- Vision measurements are added asynchronously from the Raspberry Pis over network tables
### Pathfinding
Using obstacles defined as a vertex and edge table in field.json, pathfinder will:
- Calculate valid path positions according to the clearance value (should be set at minimum to the radius of the circumcenter of the bot base)
- Calculate the shortest path from the position of the robot to a target set using the 3d field of [AdvantageScope-3044](https://github.com/FRCTeam3044/AdvantageScope-3044)
- Follow those paths with pure pursuit
### Click Drive
Driving to any point on the field from a click in the 3D field of [AdvantageScope-3044](https://github.com/FRCTeam3044/AdvantageScope-3044).

## Notes
If suddenly crashes upon simulation start delete your saved simulation layout as it may be trying to grab a field image that has been changed
Field dimensions are 16.5 by 8 and the map is calibrated accordingly, this is the same as what the april tag table uses.