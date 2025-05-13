package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class DefaultDrive extends Command {
    public DefaultDrive() {
        addRequirements(Robot.swerve);
    }

    @Override
    public void execute() {
        double x = Robot.driverControls.getLeftX();
        double y = Robot.driverControls.getLeftX();
        double rotation = Robot.driverControls.getRightX();

        if (x == 0 && y == 0 && rotation == 0) {
            Robot.swerve.stopMotors();
        } else {
            Robot.swerve.driveFieldRelative(
                    y * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    x * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    rotation * Constants.SWERVE.MAX_ANGULAR_RATE.in(RotationsPerSecond));
        }
    }
}
