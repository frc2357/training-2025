package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorMoveToDistance extends Command {

  Distance m_distance;

  public LateratorMoveToDistance(Distance distance) {
    addRequirements(Robot.laterator);
    m_distance = distance;
  }

  @Override
  public void initialize() {
    Robot.laterator.SetTargetDistance(m_distance);
  }

  @Override
  public void execute() {
    Robot.laterator.updateMotor();
  }

  @Override
  public boolean isFinished() {
    return Robot.laterator.isAtTargetDistance();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}
