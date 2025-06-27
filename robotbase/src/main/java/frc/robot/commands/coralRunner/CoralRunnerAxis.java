package frc.robot.commands.coralRunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class CoralRunnerAxis extends Command {

  Supplier<Dimensionless> m_axisSpeed;

  public CoralRunnerAxis(Supplier<Dimensionless> axisSpeed) {
    addRequirements(Robot.coralRunner);
    m_axisSpeed = axisSpeed;
  }

  @Override
  public void execute() {
    Robot.coralRunner.setAxisSpeed(m_axisSpeed.get());
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
