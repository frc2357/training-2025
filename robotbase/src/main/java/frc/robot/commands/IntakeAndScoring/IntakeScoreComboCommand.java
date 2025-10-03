package frc.robot.commands.IntakeAndScoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;

public class IntakeScoreComboCommand extends ConditionalCommand {

  public IntakeScoreComboCommand() {
    super(new ScoreAndReturn(), new Intake(), Robot.coralRunner::hasCoral);
  }
}
