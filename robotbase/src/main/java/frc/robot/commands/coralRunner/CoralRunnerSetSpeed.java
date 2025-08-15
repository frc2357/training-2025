package frc.robot.commands.coralRunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerSetSpeed extends Command {

  Dimensionless m_speed;

  public CoralRunnerSetSpeed(Dimensionless speed) {
    addRequirements(Robot.coralRunner);
    m_speed = speed;
  }

  @Override
  public void execute() {
    Robot.coralRunner.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.coralRunner.stop();
  }
}
