package frc.robot.commands.Posing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class Pose_Home extends ParallelCommandGroup {

  public Pose_Home() {
    super(
      new LateratorSetDistance(LATERATOR.SETPOINT.HOME),
      new ElevatorSetDistance(ELEVATOR.SETPOINT.HOME).beforeStarting(
        new WaitUntilCommand(Robot.laterator::isAtTarget)
      )
    );
  }
}
