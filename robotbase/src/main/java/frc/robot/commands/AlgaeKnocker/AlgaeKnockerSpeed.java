package frc.robot.commands.AlgaeKnocker;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaeKnockerSpeed extends Command {

  Dimensionless m_speed;

  public AlgaeKnockerSpeed(Dimensionless speed) {
    addRequirements(Robot.algaeKnocker);
    m_speed = speed;
  }

  @Override
  public void execute() {
    Robot.algaeKnocker.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaeKnocker.stop();
  }
}
