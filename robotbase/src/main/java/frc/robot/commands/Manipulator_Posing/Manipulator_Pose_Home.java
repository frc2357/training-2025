package frc.robot.commands.Manipulator_Posing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class Manipulator_Pose_Home extends ParallelCommandGroup {

  public Manipulator_Pose_Home() {
    super(
      new LateratorSetDistance(LATERATOR.SETPOINT.HOME),
      new ElevatorSetDistance(ELEVATOR.SETPOINT.HOME).beforeStarting(
        new WaitUntilCommand(Robot.laterator::isAtTarget)
      )
    );
  }
}
