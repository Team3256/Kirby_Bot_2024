// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexIO {
    @AutoLog
    public static class SpindexIOInputs {
        public double spindexMotorVoltage = 0.0;
        public double spindexMotorVelocity = 0.0;
        public double spindexMotorStatorCurrent = 0.0;
        public double spindexMotorSupplyCurrent = 0.0;
        public double spindexMotorTemperature = 0.0;
    }

    public default void updateInputs(SpindexIOInputs inputs) {}

    public default void setSpindexVoltage(double voltage) {}

    public default void setSpindexVelocity(double velocity) {}

    public default void off() {}
}
