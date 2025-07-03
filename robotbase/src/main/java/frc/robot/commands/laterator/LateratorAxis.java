package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class LateratorAxis extends Command {

  Supplier<Dimensionless> m_axisSpeed;

  public LateratorAxis(Supplier<Dimensionless> axisSpeed) {
    addRequirements(Robot.laterator);
    m_axisSpeed = axisSpeed;
  }

  @Override
  public void execute() {
    Robot.laterator.setAxisSpeed(m_axisSpeed.get());
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
