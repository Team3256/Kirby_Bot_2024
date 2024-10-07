import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.ControllerMapper;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

public class ControllerMapperTest {
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);
    ControllerMapper controllerMapper = new ControllerMapper(driver, operator);

    @Test
    public void testMapJsonOutput() {
        controllerMapper.bindDriver("a", "Swerve Drive Forward");
        controllerMapper.bindDriver("b", "Swerve Drive Backward");
        controllerMapper.bindDriver("x", "Swerve Strafe Left");
        controllerMapper.bindDriver("y", "Swerve Strafe Right");
        controllerMapper.bindDriver("leftBumper", "Azimuth set rotation to Source");
        controllerMapper.bindDriver("rightBumper", "Azimuth set rotation to Community");
        controllerMapper.bindDriver("start", "Reset Gyro");
        controllerMapper.bindDriver("back", "Toggle Field-Oriented Drive");

        controllerMapper.bindOperator("a", "Set Height Low");
        controllerMapper.bindOperator("b", "Set Height Medium");
        controllerMapper.bindOperator("x", "Set Height High");
        controllerMapper.bindOperator("y", "Set Height Max");
        controllerMapper.bindOperator("leftBumper", "Intake");
        controllerMapper.bindOperator("rightBumper", "Outtake");
        controllerMapper.bindOperator("start", "Start Climb");
        controllerMapper.bindOperator("back", "Abort Climb");
        String json = controllerMapper.getControllerMap();
        assertTrue(json.contains("Swerve Drive Forward"));
        assertTrue(json.contains("operator"));
        assertTrue(json.contains("drive"));
    }
}
