package frc.robot.commands.coralRunner;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerSpeed extends Command {

  Supplier<Dimensionless> m_speed;

  public CoralRunnerSpeed(Supplier<Dimensionless> speed) {

    addRequirements(Robot.coralRunner);
    m_speed = speed;
  }

  public CoralRunnerSpeed(Dimensionless speed) {
    this(() -> speed);
  }

  @Override
  public void execute() {
    Robot.coralRunner.setSpeed(m_speed.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Robot.coralRunner.stop();
  }
}
