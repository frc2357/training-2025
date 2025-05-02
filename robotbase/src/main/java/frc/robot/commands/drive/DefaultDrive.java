package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DefaultDrive extends Command {
    public DefaultDrive() {
        addRequirements(Robot.swerve);
    }

}
