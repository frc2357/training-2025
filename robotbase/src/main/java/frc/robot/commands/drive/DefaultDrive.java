package frc.robot.commands.drive;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.Supplier;

public class DefaultDrive extends Command {

  Supplier<Double> m_x;
  Supplier<Double> m_y;
  Supplier<Double> m_rotation;

  public DefaultDrive(
    Supplier<Double> x,
    Supplier<Double> y,
    Supplier<Double> rotation
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;
    m_rotation = rotation;
  }

  @Override
  public void execute() {
    if (m_x.get() == 0 && m_y.get() == 0 && m_rotation.get() == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(
        m_y.get() * Constants.SWERVE.AXIS_MAX_SPEED.in(Units.Percent),
        m_x.get() * Constants.SWERVE.AXIS_MAX_SPEED.in(Units.Percent),
        -m_rotation.get() *
        Constants.SWERVE.AXIS_MAX_ANGULAR_RATE.in(Units.Percent)
      );
    }
  }
}
