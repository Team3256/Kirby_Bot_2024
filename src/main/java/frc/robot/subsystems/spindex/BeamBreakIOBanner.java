package frc.robot.subsystems.spindex;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOBanner implements BeamBreakIO {
    private final DigitalInput beamBreak = new DigitalInput(SpindexConstants.beambreakID);

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.beamBroken = !beamBreak.get(); // i lowk forgot if this is inverted or not
    }
}
