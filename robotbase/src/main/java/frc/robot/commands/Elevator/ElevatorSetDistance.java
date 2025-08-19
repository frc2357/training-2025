package frc.robot.commands.Elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ElevatorSetDistance extends Command {

  private Supplier<Distance> m_setpoint;

  public ElevatorSetDistance(Distance setpoint) {
    addRequirements(Robot.elevator);
    m_setpoint = () -> setpoint;
  }

  public ElevatorSetDistance(Supplier<Distance> setpoint) {
    addRequirements(Robot.elevator);
    m_setpoint = setpoint;
  }

  @Override
  public void execute() {
    Robot.elevator.setTargetDistance(m_setpoint.get());
    Robot.elevator.updateMotorPIDs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.stop();
  }
}
