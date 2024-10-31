package frc.robot.utils.generics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface SingleMotorConstants {
    public static final boolean kUseFOC = true;
    public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();
    public static final int kMotorID = 0;
    public static final boolean kUseMotionMagic = true;
    public static final double kStatusSignalUpdateFrequency = 100.0; // Hz
    public static final int kFlashConfigRetries = 5;

    public static class SimulationConstants {
        public static final double kGearRatio = 1.0;
    }
}
