package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class LateratorSetDistance extends Command {

  Supplier<Distance> m_setpoint;

  public LateratorSetDistance(Distance setpoint) {
    addRequirements(Robot.laterator);
    m_setpoint = () -> setpoint;
  }

  public LateratorSetDistance(Supplier<Distance> setpoint) {
    addRequirements(Robot.laterator);
    m_setpoint = setpoint;
  }

  @Override
  public void execute() {
    Robot.laterator.setTargetDistance(m_setpoint.get());
    Robot.laterator.updateMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean isInterrupted) {
    Robot.elevator.stop();
  }
}
