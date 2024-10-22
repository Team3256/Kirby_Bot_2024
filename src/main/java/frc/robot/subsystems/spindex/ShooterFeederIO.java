package frc.robot.subsystems.spindex;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFeederIO {
    @AutoLog
    public static class ShooterFeederIOInputs {
        public double shooterFeederMotorVoltage = 0.0;
        public double shooterFeederMotorVelocity = 0.0;
        public double shooterFeederMotorStatorCurrent = 0.0;
        public double shooterFeederMotorSupplyCurrent = 0.0;
        public double shooterFeederMotorTemperature = 0.0;
    }

    public default void updateInputs(ShooterFeederIOInputs inputs) {}

    public default void setFeederVoltage(double voltage) {}

    public default void setFeederVelocity(double velocity) {}

    public default void off() {}
}
