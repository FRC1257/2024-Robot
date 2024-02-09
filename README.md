Superstructure of all our robot's subsystems

# 1257 Robot Code
This project has Parallel Universe's template robot code.

## Features
### Advantage Kit
[Advantage Kit](https://github.com/Mechanical-Advantage/AdvantageKit) improves our logging, simulation, and readibility. All of our subsystems are separated into simulation and real IO modes. The subsystem logic is stored inside the `Subsystem.java` classes. The mode that the robot code is in can be changed by going to `Constants.java`.  

### Drive
We use a swerve ([REV Max Swerve](https://docs.revrobotics.com/ion-build-system/motion/maxswerve)) drivetrain with various different IO options like simulation and Spark Max motor controllers. We have some basic arcade drive, trajectory following, and turn angle commands. 

#### Path Planner
We use [Path Planner](https://pathplanner.dev/) to create and follow our autos. We also use it to generate trajectories on the fly to go to setpoints on the field. Currently, we are also working on implementing path finding as well. We also plan to use [Choreo](https://sleipnirgroup.github.io/Choreo/) to create better paths in the future. 

### Vision
Our team uses [Photon Vision](https://photonvision.org/) with a Raspberry Pi 4 to detect April Tags for pose estimation and as a driver camera. The `Vision` folder has the simulation and real components for this system. 

#### Auto Aiming
We also have a command called `joystickSpeakerPoint` in our `DriveCommands.java` file that we use to aim from any position on the field to the speaker.

### Auto
Last season, we had a customizable auto choosing system. Currently, this code is being rewritten, but is stored in the `util` folder. Some of the code is going to be manually created Path Planner autos, while the others will be generated programmatically.

## Getting Started
Take a look at the resources we have [here](https://docs.google.com/document/d/1KaAQCZHfttFZk9dY0amIj057UGiLFXdztHYZKqD3VwI/edit#heading=h.8op83lvrsd9) with things we need to do.

First, install [WPILib](https://docs.wpilib.org/en/stable/docs/software/what-is-wpilib.html) and clone this repository.

> We recommend using [GitHub Desktop](https://desktop.github.com/) for this

Once you open VSCode, you will see our project. If you see red squiggly lines in certain files, don't worry about that. Press the WPILib icon in the top right and press `Build Robot Code`. Once you do so, it will download all the vendor dependencies and generate the classes we need to run our code.

> Notice that two new folders called `.gradle` and `build` get created. That's what the build command generated to prepare all of our code.

To see if your code runs, press the WPILib icon then press `Simulate Robot Code`. Then press the `Sim GUI` extension. From here, you can drag in your joysticks into the simulation area (if you are on a laptop, you can use your keyboard as a controller instead). Then open either [Advantage Scope](https://github.com/Mechanical-Advantage/AdvantageScope) or [Shuffleboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html) and connect to the simulated robot. 

> We recommend using Advantage Scope since our code is written in the Advantage Kit framework and there are way more cool features!

In our project, we have a Advantage Scope layout for our robot called `AdvantageScope-layout.json` in the `advantage_scope` folder. In Advantage Scope, open this layout and you will see all of the [different visulizations](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/NAVIGATION.md) of the data from our robot that we have. Back in simulation, if you try moving around our robot, you will also see it moving on the [3D field](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md). 

That's cool, but do you know what would be even cooler? If we could have our robot cad on screen!!! In Advantage Scope, navigate to the `Help > Show Assets Folder` button. This will create a pop-up showing you where the custom assets used in Advantage Scope can be added. Copy the folder called `Robot_AllenV1` from the `advantage_scope` folder and into the custom assets folder. Once you do this, in the 3D field, you will be able to change your robot to Allen and drive around our robot on the field!

Last thing. If you want to see what the simulated vision system sees, navigate to the `CameraPublisher` in the Network Table (this should be in the left tab section in Advantage Scope and Shuffleboard). Here you will see all of the cameras currently connected to the robot. Click into one of them and go to `Camera-processed` then `streams`. Here you will see the location where your camera is getting streamed.

> In simulation, it will most likely look something like this `http://localhost:1182/?action=stream`

Awesome right!

## Sources
Much of our code has come from various example projects from across the internet. Since we use Advantage Kit, we looked mainly at these repositories for inspiration: [Mechanical Advantage 2024 Kitbot Swerve Project](https://github.com/Mechanical-Advantage/RobotCode2024/tree/kitbot-swerve), [Advantage Kit Advanced Swerve Project](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main), [Mechanical Advantage 2023 Robot Code](https://github.com/Mechanical-Advantage/RobotCode2023), [Our 2023 Virtual Robot Project](https://github.com/frc1257/virtual-robot), [Path Planner Example Project](https://github.com/mjansen4857/pathplanner/tree/main/examples/java), as well as [Photon Vision Example Project](https://github.com/PhotonVision/photonvision/tree/master/photonlib-java-examples). We hope these sources help you out as well!

