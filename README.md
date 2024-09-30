# Kirby_Bot_2024

A cute pink circular turret bot, competing at CalGames 2024. 

## Code structure

Since we're using AdvantageKit, we follow the [IO-layer](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md)-based, [command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html), project structure [similar to MechanicalAdvantage](https://github.com/Mechanical-Advantage/RobotCode2022/tree/main/src/main/java/frc/robot/subsystems).

The dependencies we use are (besides [WPILIB](https://docs.wpilib.org/en/stable/index.html)):

- [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit): logging library (saved to file)
- [Monologue](https://github.com/shueja/Monologue): also a logging library, but for sending things to NT for the dashboard
- [Phoenix 6](https://api.ctr-electronics.com/phoenix6/release/java/) & [5](https://api.ctr-electronics.com/phoenix/release/java/) (for the [CANdle](https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html)): library for interfacing with CTRE hardware
  - [CTRE generated swerve](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html)
- [Choreo](https://github.com/SleipnirGroup/Choreo): optimized auto path, generated ahead of time
- [Pathplanner](https://pathplanner.dev/home.html): auto path composition & dynamic path generation
- ..and [Limelight](https://limelightvision.io/)'s [single-file library](https://github.com/LimelightVision/limelightlib-wpijava)

Other than the typical FRC code structure scaffold, we also have GitHub Actions and some scripts to help with deployment.

## Style guide

We use Spotless for formatting (run `./gradlew spotlessApply`) and follow [conventional commits](https://www.conventionalcommits.org/en/v1.0.0/).

We also have a very nuanced format for doing configs. An example will come soon! See [here](https://github.com/Team3256/Offseason_Bot_2024/blob/6bcbbed94f5959c411d5b3c4f46dded32fb10a72/src/main/java/frc/robot/subsystems/ampbar/AmpBarConstants.java#L28) for now.
