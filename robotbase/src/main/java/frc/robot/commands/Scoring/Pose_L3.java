package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class Pose_L3 extends SequentialCommandGroup {

  public Pose_L3() {
    super(
      new ElevatorSetDistance(ELEVATOR.SETPOINT.L3),
      new LateratorSetDistance(LATERATOR.SETPOINT.L3)
    );
  }
}
