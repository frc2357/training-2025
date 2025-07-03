package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorSpeed extends Command {

  Dimensionless m_speed;

  public LateratorSpeed(Dimensionless speed) {
    addRequirements(Robot.laterator);
    m_speed = speed;
  }

  @Override
  public void execute() {
    Robot.laterator.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}
