# Arm Subsystem Example
Use this as a reference when programming and arm in the future!

## What it does:
This is just a full, can be simulated, arm subsystem in WPILib. It contains all the information you should know
and what you should do to program an arm. It uses Kraken x60 and CTRE MotionMagic©

## High Level Overview:
In ArmSubsystem.java

### Initialize all your objects:
 - Arm motor
 - Arm motor (Sim)
 - Motor Control
 - Motor config
 - Mechanism2d
 - Mechanism root
 - Mechanism Ligament
 - SingleJointedArm physics simulation

### Constructor:
Configure motor:
 - configure PID
 - set current limits (supply and stator)
 - configure motion magic control parameters
 - Set the gear ratio
 - state the feedback source
 - state the brake mode
 - state the inversion mode
 - apply config

### Periodic (Happens once every 20msec):
 - Put data on SmartDashBoard, especially the arm's Mechanism2d

### SimulationPeriodic(Same as periodic but only happens in simulation mode)
 - set supply voltage to 12 volts
 - set the input to the arm physics simulation
 - update it with a delay of 0.2 secs (20msec)
 - set the rotor position and velocity for simulated motors using data from the physics simulation
 - set the ligament angle to the physics simulation angle

### Some helper methods
 - getArmPositionDegrees()
 - getArmSetpointDegrees()
 - hasReachedAngle()

### Setting arm position
 - clamp the input to the min/max range
 - set the control parameters, such as the position to go to and the slot to use
 - set the motor's control to the control object

## Challenge
Can you make the arm use an external CANCoder instead of the internal rotor encoder?