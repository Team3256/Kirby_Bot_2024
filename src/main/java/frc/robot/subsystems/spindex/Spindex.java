package frc.robot.subsystems.spindex;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

    
// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

public class Spindex extends SubsystemBase {
    private TalonFX newMotor = new TalonFX(SpindexConstants.spindexMotorID);
    private DigitalInput beamBreak = new DigitalInput(SpindexConstants.spindexMotorID)
    // private beambreak Beambreak = new beamreak(1)
    public Spindex() {}


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command spin () {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
       () -> { 
          newMotor.set(SpindexConstants.spindexMotorSpeed);
        }).finallyDo(this::stop).until(beamBreak::get);
  }
  public void stop() {
    newMotor.set(0);
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}