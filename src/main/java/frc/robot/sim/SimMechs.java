package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public final class SimMechs {

    public static final Mechanism2d mech = new Mechanism2d(5,5);

    private static final MechanismRoot2d ampevator = mech.getRoot("ampevator", 1,0);

    private static final MechanismRoot2d pivot = mech.getRoot("pivot", 4,0);

    private static final MechanismLigament2d m_ampevator = ampevator.append(new MechanismLigament2d("ampevator", 2, 90));
    private static final MechanismLigament2d m_ampevatorRollers = m_ampevator.append(new MechanismLigament2d("rollers", 1, 0));
}
