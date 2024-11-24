>[!NOTE]
> This repository has been archived. No further development on Kirby will continue.

# Kirby_Bot_2024

A cute pink circular turret bot, competing at Madtown Throwdown 2024. 

## Code structure

Since we're using AdvantageKit, we follow the [IO-layer](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md)-based, [command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html), project structure [similar to Highlander Robotics](https://github.com/HighlanderRobotics/Crescendo).

The dependencies we use are (besides [WPILib](https://docs.wpilib.org/en/stable/index.html)):

- [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit): logging library (saved to file)
- [Monologue](https://github.com/shueja/Monologue): also a logging library, but for sending things to NT for the dashboard
- [Phoenix 6](https://api.ctr-electronics.com/phoenix6/release/java/) & [5](https://api.ctr-electronics.com/phoenix/release/java/) (for the [CANdle](https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html)): library for interfacing with CTRE hardware
  - [CTRE generated swerve](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html)
- [Choreo](https://github.com/SleipnirGroup/Choreo): optimized auto path, generated ahead of time
- [Pathplanner](https://pathplanner.dev/home.html): dynamic path generation
- ..and [Limelight](https://limelightvision.io/)'s [single-file library](https://github.com/LimelightVision/limelightlib-wpijava)

Other than the typical FRC code structure scaffold, we also have GitHub Actions and some scripts to help with deployment.


### Constants file and feature flags

I'm a bit pendantic about this because I care about consistence (I don't have OCD I swear) so here's the structure:

- We have our subsystem-specific constants (`SubsystemConstants.java`) and configs decentralized. This includes
  - Motor configs (e.g. gain values and current limits)
  - Subsystem characteristics (for physics simulation and gear ratios)
  - Position/velocity presets
  - `kUse___` = enables/disables a specific feature in the subsystem
- We have our feature flags and otherwise global constants (e.g. logging and controller constants) in the `Constants.java` file
  - `k___Enabled` = enables/disables a subsystem
  - Enabling features of a single subsystem should be within the decentralized subsystem constants file (unless it is a feature that spans across multiple subsystems)
- Don't write feature flags for things that you could just straight up not use. Feature flags is the non-scuffed way of "temporarily disabling this code by commenting it out"
  - If you ever find yourself needing to comment some code out, consider creating a feature flag
- Have the feature flag-checking logic outside of your function (or, if it's a subsystem, passed in as a boolean value of `enabled` for that subsystem's `DisabledSubsystem` constructor)


## How to




### ...start developing??

```shell
git clone https://github.com/Team3256/Kirby_Bot_2024.git
cd Kirby_Bot_2024
# if any of these commands fail, you either
# installed WPILib incorrectly
# or you didn't add ~/wpilib/2024/jdk/bin
./gradlew wrapper
./gradlew build
# or, if you want to simulate teh code
./gradlew simulateJava
```


Gradle build will automatically run checkers to make sure there are no null pointers within robot container that could cause the robot code to crash when deployed on the robot but it isn't fool-proof so you should still simulate changes to make sure the code doesn't crash. 

### ...run the formatter?

```shell
./gradlew spotlessApply
```

## ...generate controller maps?

```shell
./gradlew simulateJava
# Running simulateJava will dump the controller map onto a file. Quit it right after the program starts
python scripts/gen_controller_map.py
# Now, open the index.html
# (`open` is a macOS-only command)
open scripts/index.html
```

### Developing during comp

Ideally, you're not making any changes during comp but just in case...
Branch off main (or the most recent working development branch) and name the branch "event/(the event name)". This will ensure that a commit is created whenever robot code is deployed. After any changes, try simulating and going into both auto and teleOP to make sure nothing crashes. Once you deploy make sure you run either a systems check or if you do not have time check the driver station to make sure the robot code has at least booted up. 


## Style guide

Check out the WB software manual for some styling stuff: https://docs.google.com/document/d/1stGoEnXgRHY7ff4rGbEselquu_KpRetePAEhrPY2oH0/edit

We use Spotless for formatting (run `./gradlew spotlessApply`) and follow [conventional commits](https://www.conventionalcommits.org/en/v1.0.0/).

We also have a very nuanced format for doing configs. An example will come soon! See [here](https://github.com/Team3256/Offseason_Bot_2024/blob/6bcbbed94f5959c411d5b3c4f46dded32fb10a72/src/main/java/frc/robot/subsystems/ampbar/AmpBarConstants.java#L28) for now.
