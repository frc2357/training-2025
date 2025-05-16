package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class DefaultDrive extends Command {
    public DefaultDrive() {
        addRequirements(Robot.swerve);
    }

    @Override
    public void execute() {
        double x = Robot.driverControls.getLeftX();
        double y = Robot.driverControls.getLeftY();
        double rotation = -Robot.driverControls.getRightX();

        System.out.println(x);

        if (x == 0 && y == 0 && rotation == 0) {
            Robot.swerve.stopMotors();
        } else {
            Robot.swerve.driveFieldRelative(
                    y * Constants.SWERVE.AXIS_MAX_SPEED,
                    x * Constants.SWERVE.AXIS_MAX_SPEED,
                    rotation * Constants.SWERVE.AXIS_MAX_ANGULAR_RATE);
        }
    }
}
