
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;

public class LineFollower extends Command {

    AnalogInput m_followerRight;
    Drivetrain m_drivetrain;

    public LineFollower(Drivetrain drivetrain) {
        m_followerRight = new AnalogInput(0);
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);

    }

    public void execute() {
        System.out.println(m_followerRight.getVoltage());
        m_drivetrain.arcadeDrive(1, 0);
    }

    public boolean isFinished() {
        return m_followerRight.getVoltage() >= 4;

    }

    public void end() {
        m_drivetrain.arcadeDrive(0, 0);
    }
}
