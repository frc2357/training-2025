package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class LateratorSetpoint extends Command {

  Supplier<Distance> m_setpoint;

  public LateratorSetpoint(Distance setpoint) {
    addRequirements(Robot.laterator);
    m_setpoint = () -> setpoint;
  }

  public LateratorSetpoint(Supplier<Distance> setpoint) {
    addRequirements(Robot.laterator);
    m_setpoint = setpoint;
  }

  @Override
  public void initialize() {
    Robot.laterator.setTargetDistance(m_setpoint.get());
  }

  @Override
  public void execute() {
    Robot.laterator.updateMotor();
  }

  @Override
  public boolean isFinished() {
    return Robot.laterator.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}
