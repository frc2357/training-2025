package frc.robot.commands.coralRunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerAxis extends Command {
    double m_axisSpeed;

    public CoralRunnerAxis(double axisSpeed) {
        addRequirements(Robot.coralRunner);
        m_axisSpeed = axisSpeed;
    }

    @Override
    public void execute() {
        Robot.coralRunner.setAxisSpeed(m_axisSpeed);
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
