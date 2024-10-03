package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;

// Singleton for simulation visualization
public enum SimViz implements Subsystem {
    INSTANCE;

    private static final Mechanism2d mech = new Mechanism2d(1, 1);
    private static final MechanismRoot2d shooterAxle = mech.getRoot("Shooter Axle", 0.0, 0.0);
    private static final MechanismLigament2d shooterVis = shooterAxle.append(
            new MechanismLigament2d("Shooter Flywheel", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));

    private SimViz() {
        System.out.println("Here");
    }

    public static SimViz getInstance() {
        SmartDashboard.putData("/SimViz", mech);
        return INSTANCE;
    }

    // angle is in degrees
    public void addToShooterFlywheelAngle(double angle) {
        shooterVis.setAngle(shooterVis.getAngle() + angle);
    }

}
