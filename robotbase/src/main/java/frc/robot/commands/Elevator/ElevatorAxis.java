package frc.robot.commands.Elevator;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ElevatorAxis extends Command {

  Supplier<Dimensionless> m_axisSpeed;

  public ElevatorAxis(Supplier<Dimensionless> axisSpeed) {
    addRequirements(Robot.elevator);
    m_axisSpeed = axisSpeed;
  }

  @Override
  public void execute() {
    Robot.elevator.setAxisSpeed(m_axisSpeed.get());
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
