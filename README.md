# 1257 Robot Code
This project has Parallel Universe's template robot code.

## Features
### Advantage Kit
[Advantage Kit](https://github.com/Mechanical-Advantage/AdvantageKit) improves our logging, simulation, and readibility. All of our subsystems are separated into simulation and real IO modes. The subsystem logic is stored inside the `Subsystem.java` classes. The mode that the robot code is in can be changed by going to `Constants.java`.  

### Drive
We use a tank drivetrain with various different IO options like simulation, Spark Max with NEOs, Spark Max with CIMs, and Talon motor controllers. We have some basic arcade drive, trajectory following, and turn angle commands. 

### Vision
Our team uses [Photon Vision](https://photonvision.org/) with a Raspberry Pi 4 to detect April Tags for pose estimation and as a driver camera. The `Vision` folder has the simulation and real components for this system. 

### Auto
Last season, we had a customizable auto choosing system. Currently, this code is being rewritten, but is stored in the `util` folder.

## Getting Started
Take a look at the resources we have [here](https://docs.google.com/document/d/1KaAQCZHfttFZk9dY0amIj057UGiLFXdztHYZKqD3VwI/edit#heading=h.8op83lvrsd9) with things we need to do.

First, install [WPILib](https://docs.wpilib.org/en/stable/docs/software/what-is-wpilib.html) and clone this repository.

> We recommend using [GitHub Desktop](https://desktop.github.com/) for this

Then in VSCode, press the WPILib icon in the top right and simulate. Then you will be able to watch your code run!