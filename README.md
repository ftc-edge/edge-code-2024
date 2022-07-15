## Welcome!
This repository is a fork of the official FTC SDK from https://github.com/FIRST-Tech-Challenge/FtcRobotController. It has the addition of our Titan Robotics Framework Library (TrcLib) and its dependencies. These are added as git submodules so that they can be independently managed without the need of changing the FTC SDK (except for minor gradle changes to tie in our Framework Library dependencies). Therefore, one can update the FTC SDK by pulling changes from the upstream FTC SDK without affecting our Framework Library. You can also pick up the latest changes of our Framework Library without affecting the FTC SDK. It serves as a clean repository template for the start of a new season. This template also contains basic team code that implements a mecanum drive base with teleop control. It allows you to run a simple mecanum robot in TeleOp almost right out of the box. In addition, it includes autonomous infrastructure code that uses our FtcChoiceMenu for selecting autonomous strategies and other autonomous choice opotions. This allows you to write only one autonomous opmode to handle many permutations of autonomous. For example, one autonomous opmode can handle both red and blue alliances. The template also provides a rich set of Tests for diagnosing robot hardware and/or for tuning.

## Getting Started
To use this repository template, you can clone or fork this template repository to your own github repositories. There are many ways to do it but I am going to describe one way which is using GitHub Desktop. If you are more familiar with other similar tools, feel free to use it instead.
* On a web browser, enter the URL https://github.com/trc492/FtcTemplate.
* Then, click the "Fork" button near the upper right corner of the web page and answer the question on where you want to fork this repository to your GitHub repositories.
* Once the fork is done. Clone your GitHub repository to your computer using your favorite GitHub tool such as GitHub Desktop.
* On GitHub Desktop, click File->Clone repository..., select the repository you just forked and type in the path on the computer where you want to clone the repository to.

Congratulations! You just clone our template repository. Now you can fire up Android Studio and import this gradle project. Once that is done, you can now go to TeamCode and browse around the provided template code or sample code. You can compile the code and check if you have any issues with the cloned template. Or you can jump right in and start modifying/customizing the code.

### TeleOp Driving a Mecanum Robot Right Out Of The Box
Since this template already contains basic code for a mecanum robot base, it takes very few modifications to make it work with your mecanum robot.
* In RobotParams.java, update the string constants HWNAME_xxxx_WHEELS corresponding to the hardware names of the four driving wheels in your robot config.
* Compile the code and deploy it to the robot.
* Place your robot on a stand so that the wheels can be free running without the robot running away from you. When looking down on the robot, the mecanum wheel rollers should form an X. If not, switch the mecanum wheels around until they form an X. On the Driver Station, activate your robot configuration. Select a TeleOp Opmode on the Driver Station called FtcTest. Press the init button to initialize the opmode. Press the D-pad down button on your operator gamepad until the "Test" shown on the Driver Station is on "Drive motors test". Press the D-pad right button to select the test. Then press the "Play" button on the Driver Station to start the opmode. This test will run each of the four driving wheels one after the other for 5 seconds each in the sequence of Left Front, Right Front, Left Back and Right Back. Note the rotation direction of each wheel and make sure they would have run the robot in the forward direction if the robot were placed on the ground. If any of the wheels are rotating in the wrong direction, correct them in RobotParams.java. Change LEFT_WHEEL_INVERTED or RIGHT_WHEEL_INVERTED from true to false or vice versa to reverse the left or right side of the driving wheels until the test shows all four wheels rotating in the correct direction.

That's it. Your robot is now ready to be driven in TeleOp mode. The default drive mode is "Arcade Mode". It means the right stick on the driver gamepad controls the X and Y direction of the robot. The X-axis of the left stick controls the rotation. If you prefer, you can change the drive mode to "Holonomic Mode". In this mode, the Y-axis of the right stick controls the Y direction of the robot. The X-axis of the left stick controls the X direction, the left trigger controls turning left and the right trigger controls turning right. To change to this mode, change ROBOT_DRIVE_MODE in RobotParams.java to DriveMode.HOLONOMIC_MODE.

In addition, there are a few more buttons on the driver gamepad that modify how the robot is driven. Click the left bumper on the driver gamepad will toggle Inverted mode. Inverted mode allows the robot to switch the front and back end. This is useful for a robot that has an end effector such as an intake at the back so that the drivers can drive the robot around as if the intake is in front. Press and hold the right bumper allows you to drive the robot at half speed. This is useful for delicate movement of the robot.

## Library Features
The Framework Library provides numerous features. We will list some of them here:
- FtcOpMode: Our own opmode that extends LinearOpMode but providing interface similar to OpMode where you put your code in some sort of loop method. FtcOpMode is a cooperative multi-tasking scheduler. As an advanced feature, our Framework Library also supports multi-threaded true multi-tasking. But for rookie teams who don't want to tackle the gotchas of true multi-tasking, cooperative multi-tasking is the way to go. This allows your autonomous to operate multiple subsystems at the same time instead of doing things sequentially. This is especially important since FTC autonomous period lasts only 30 seconds. In order to perform the maximum number of tasks in the autonomous period, your code would want to perform multiple tasks that have no dependencies on each other and perform them simultaneously. The Framework Library enables that in a trivial manor.
- State machine: The state machine infrastructure is the core of multi-tasking. Each task should use a state machine to keep track of their states. This allows FtcOpMode to switch between tasks and be able to maintain the state of each task when they are resumed from suspended state.
- Task syncrhonization: Some tasks have dependencies on each other. For example, autonomous may want to finish driving the robot to the specified location before dumping the game element to the proper spot. This requires task synchronization. The Framework Library provides a number of task synchronization features such as Events (TrcEvent) and Callbacks (TrcNotifier). Event allows an operation to signal it when the operation is completed so that the task waiting for it can resume. Callback allows the task to be called to perform additional work without the use of a state machine after the operation is completed, for example.
- Timers: The Timer Manager (TrcTimerMgr) manages multiple simultaneous timers. When a timer expires, you have the option of signaling an event or do a notification callback. For example, if you want to spin a motor for 3 seconds and turn it off afterwards, you can arm a timer that expires in 3 seconds and do a notification callback to turn the motor off. This type of operation is sometimes called "fire and forget".
- Advanced multi-tasking: In addition to cooperative multi-task, our Framework Library also supports multi-threaded tasks. The Library provides a number of standard threads (i.e. main robot thread, input thread and output thread). The main robot thread runs the FtcOpMode where the scheduler is performing cooperative multi-tasking on the main robot thread. The input thread handles all input tasks such as reading sensors and odometry. The output thread handles all output tasks such as motor and actuator tasks including PID control and pathing. If there is a special need that either requires high frequency processing and cannot afford any latency or a task that takes extra long time to run and thus blocking the thread unnecessarily long, the Library enables you to create STANDALONE tasks that have their own thread. All these tasks/threads are mananged by the Task Manager (TrcTaskMgr). Although everything provided and maintained by the Library are thread-safe, user must still be cautious when writing multi-threading code. Care must be taken to ensure you don't fall into the trap of two common multi-threaded programming woes: shared resource contention and task synchronization. If you don't understand these, it's better not to do multi-threaded multi-tasking and stick only with cooperative multi-tasking. Even with cooperative multi-tasking, you still need to apply some simple disciplines: do not block the main robot thread (i.e. no busy wait loop and sleep statements in your task code). Just start an operation and get out. Do not wait for it to complete. Most of the operations supported by the Framework Library are asynchronous. Calling them will start the operation and the control is returned back to your code immediately. Your state machine should take care waiting for an operation to complete. That's why it's called "cooperative multi-tasking". You must be good citizens for it to work properly.
- Inputs: The Framework Library supports many different input devices such as gamepad controllers, sensors and driver station dashboard.
  - Gamepad controller: The Framework Library monitor all buttons on the gamepad for state changes. Any button presses or releases will result in a notification callback to your button event handler. This simplifies your TeleOp code tremendously.
  - Sensors: The Framework Library supports many type of sensors whether they are digital sensors, analog sensors, I2C sensors or even Android built-in sensors, the Library provides access to them. Popular sensors include ultronic sensor, color sensor, distance sensor such as Lidar, gyro, accelerometer, touch sensor, IR seeker and Pixy camera etc. It also supports many underlying communication protocols (e.g. I2C, Serial, SPI etc) so that you can write custom sensor driver code to communicate with sensors that the Library does not have built-in support.
  - Driver station dashboard: The Framework Library provides easy access to the driver station as an input device. It allows you to create Choice Menus (FtcChoiceMenu) or Value Menus (FtcValueMenus) where you can ask the user to provide information before the competition match is started. Information such as whether you are on the RED or BLUE Alliance; which starting position your robot is in; whether your robot should delay starting the autonomous routine to let your alliance partner to go first to avoid potential collision or whether your robot should perform or skip a certain autonomous tasks. The Choice and Value menus form a decision tree that allow the user to select choices or enter values using the gamepad buttons.
- Data Filters and Processors: In the real world, sensors have noises. In some applications, noises are bad for robot control. The Framework Library provides a number of popular data filters (e.g. IIR, Kalman and Spurious filters etc) that will clean up noises on sensor readings. It also provides some data converters such as data integrator. For example, some simple gyro sensors only give you rotational rate but not heading. You must integrate the rotational rate over time to calculate heading. In this situation, the Library provides the data integrator. In some other sensors such as some gyros or compasses, they give you non-contiguous readings when passing through some point such as the REV IMU gyro goes from 179 degrees to -180, or compass goes from 359 degrees back to zero. This non-contiguous values may cause havoc in control algorithms. In this case, the Library provides a converter that can monitor the sensor crossing such points and convert the values into a contiguous scale.
- Outputs: The Framework Library supports many types of output devices such as motors, servo, complex actuators, lights and sound.
  - Motors: Motor is the most fundamental output device on a robot. It provides movements for the robot. FTC SDK provides some basic motor classes (e.g. DcMotor, DcMotorEx). The Framework Library adds a lot more features on top of that in different layers of complexity. For example, it provides FtcDcMotor that added support for a digital input sensor to reset the motor encoder automatically. This is useful for using the motor in a complex actuator such as an arm or elevator when you may need to zero calibrate the zero position of the actuator using a lower limit switch. It also added support to provide velocity mode control and motor odometry. On top of the fundamental motor class, it also provided a PID Controlled Motor class (TrcPidMotor). This class added support for PID control, lower and upper limit switches, motor stall protection for safety, dual motor with synchronization, zero position calibration and gravity compensation. These advanced features made it trivial to implement complex subsystems such as a swing arm or elevator. The built-in PIDF controller allows the arm or elevator to be controlled by an analog joystick to speed up or slow down the arm/elevator movement. It understands the arm/elevator position approaching the lower/upper position limits and will automatically slow down its movement. It also provides stall protection. If the PID Actuator got stuck and the motor is stalled, it can cut motor power to prevent it from burning out. It also allows a reset timeout so that the stall condition can be cleared after a certain period assuming the stall condition is caused by human temporarily. This allows the subsystem to resume its function and provides time for the motor to cool down.
  - Servos: With the limited number of motors allowed by FTC, servo motors become the secondary important actuator on a robot. The Framework Library provides the basic support of a servo over the FTC SDK (TrcServo). It supports translation between logical servo positions (between the value of 0 and 1) to physical positions such as 0 to 180 degrees. Just like motors, it also allows you to invert the direction of the servo movement. On top of the TrcServo class, it also provides an Enhanced Servo class (TrcEnhancedServo). It provides the features to support dual servos, continuous servo with optional lower and upper limit switches. Most importantly, it allows speed controlling a servo motor so you can control a servo by an analog joystick.
  - Light: The Framework Library supports many ways to control lights, usually LEDs. They could be a single LED with just one color, a single RGB LED or a colr LED strip that is pixel addressable. The LED lights can be controlled by digital output ports or most likely the REV Blinkin LED controller. This is not only for asethetics to make our robot look pretty, it also serves a very practical purpose: providing feedback to drivers on robot status. For example, it can tell you whether robot vision detected the target by changing the LED color on different target positions. The Library not only allows you to lit up LEDs in different colors or different color patterns, it also provides a complex priority scheme in controlling the LEDs. Imagine the robot has many subsystems that want to tell you something. Vision may want to tell you whether it sees the target, intake may want to tell you whether it has taken in a game piece. All these events will cause LED contention (i.e. different subsystems fighting to change the color of the LEDs to show their status). The Library provides a priority scheme (TrcPriorityIndicator) that defines what color pattern has priority over the others so that important events can override lower priority events to show their status. The Library also provides support for LED Matrix Panel. Although it is not really practical for FTC, it is more an FRC feature because of power requirement and also requires custom electronics that FTC may not allow.
  - Sound: The Framework Library supports sound. It used to support sound on the Robot Controller when it was an Android Phone. With the introduction of REV Robot Controller Hub, it no longer provides sound capability. The sound support now gets redirected to the Driver Station. Sound support includes playing a tone or providing text to speech. Sound is an important output device. Just like light, it provides feedback to the drivers on important robot status. For example, if one of the motors is stalled, the Library can cut power to the motor to prevent it from burning out. Sound support can generate a beep to warn the driver about it.
- Drive Base: Our Framework Library provided support for 3 different types of drive bases: Simple Drive Base (TrcSimpleDriveBase), Mecanum Drive Base (TrcMecanumDriveBase) and Swerve Drive Base (TrcSwerveDriveBase). All drive bases have built-in odometry and localization support. It means the library can use sensors such as wheel encoders and gyro to keep track of the absolute field location of the robot. It even supports passive-wheel odometry (aka dead-wheel odometry) where it supports 2 to 4 passive omniwheels with encoders to keep track of the absolute field location of the robot. All odometry data can be scaled to real world unit such as inches instead of encoder counts. All drive bases support many advanced features such as stall detection that detects if the drive base is stalled because it runs into an obstacle. This allows the possibility of writing advanced code such as running into the field wall to relocalize the robot location. Running into the field wall causes the drive base to stall which can signal an event to stop the drive base and reset the robot location to a known position. All drive bases provide support for different drive strategies such as tankDrive, arcadeDrive and curveDrive. It also provides support for holonomicDrive for drive bases that have this capability such as Mecanum and Swerve Drive Bases.
  - Simple Drive Base supports drive bases with 2 to 6 motors. It has left and right sides. Each side can have 1 to 3 motors. Simple Drive Base can only run straight and cannot strafe like Mecanum Drive Bases (i.e. no holonomicDrive support).
  - Mecanum Drive Base has 4 mecanum wheels. Each wheel has its own motor. It is capable of holonomic drive (i.e. strafing).
  - Swerve Drive Base has 4 swerve wheel modules. Each swerve module consists of a drive motor and a steering motor. The steering motor can be a DC motor or a servo motor. Each swerve module on a Swerve Drive Base can be independently steer so that it can run in any direction with the robot heading pointing to a totally different direction.
- Exclusive Subsystem: A robot consists of many subsystems (e.g. drive base, elevator, shooter, intake, vision etc). Most of the subsystems can be operated by human operator in TeleOp mode. However, some subsystems can also be used in auto-assist operations. For example, on the press of a button, the vision-assisted shooter may stop the drive base, acquire a camera image for vision processing, calculate the trajectory for shooting a target, spin the shooter up to shooting speed, pan and tilt the shooter to the correct angles aiming at the target and shoot. The auto-assist operation involves several subsystems while these subsystems can also be operated by human control. Without coordination, human control and auto-assist may fight each other for the control of these subsystems. For example, while auto-assist is tilting the shooter for aiming the target, the tilting mechanism is also controlled by a joystick which is at neutral position. It means teleop code is constantly sending zero power to the tilter while auto-assist is trying to move the tilter. This causes jerky movement of the tilter. With Exclusive Subsystem support, one can declare the tilter as an Exclusive Subsystem. Before auto-assist starts an operation with the Exclusive Subsystem, it must acquire ownership of the subsystem. Once ownership is acquired, nobody else except the owner can operate the subsystem. This prevents teleop control from interfering with the auto-assist operation. When the auto-assist operation is done, the exclusive ownership of the subsystems will be released so that telop control can be resumed.
- PID Control:
- Odometry:
- Pathing:
- Vision: In every season, the game play usually involves navigating the robot to a certain location marked by objects detectable by vision or the robot is shooting game elements at a target recognizable by vision. Computer vision can have different complexity, ranging from simple color blob detection to full blown neural network object recognition. Therefore, it is generally computational intensive and could take hundreds of milliseconds to process an image frame. Fortunately, there are industrial vision libraries that take care of the heavy lifting for us. Libraries such as Vuforia, Tensor Flow and OpenCV. Our Framework Library includes support for all these industrial libraries. Some industrial libraries support asynchronous processing of image frames and some do not. For those that don't, they could block our main robot thread for hundreds of milliseconds. But with our Framework Library, we wrap these industrial libraries with easy to use interfaces and provide asynchronous support thus freeing our main robot thread to take care of other tasks. This encapsulation makes it extremely easy to writing vision code.
- Util:
  - Trace logging: In addition of providing information output to the dashboard on the driver station, the Framework Library also provides trace logging mainly for debugging purpose. It is a super important tool allowing post-mortem analysis of the robot performance of a match either for debugging or for performance tuning. Information written to the trace log has different levels that can be adjusted to reduce clutter. The levels are: VERBOSE, INFO, WARNING, ERROR and FATAL. For example, in regular competition match log, we will only turn up to the INFO level at the most. But for debugging, we may turn up to VERBOSE to see everything.

## Getting Help
### User Documentation and Tutorials
We are not very good at creating documentation and tutorials but we want to get better at this. Our Framework Library code has JavaDoc all over. Therefore, you can get information on what each class does and their methods. We have also added sample code to the template project. Several teams have been using our Framework Library and we welcome opportunities of collaboration in creating tutorial materials. Feel free to suggest what tutorial you want to see.

### Javadoc Reference Material
The Javadoc reference documentation for the TRC Robotics Framework Library can be found [here](https://trc492.github.io/FtcJavaDoc/).

### Online User Forum
For technical questions regarding our Framework Library, please post questions on the FTC Forums [here](https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio).

### Sample OpModes

# Release Information

## Version 1.0 (2021-11-24)

### Enhancements and New Features

### Bug fixes
