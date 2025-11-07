package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class StopAtDistance extends Command {

    AnalogInput m_rangeFinder;
    Debouncer m_boing;
    Drivetrain m_drivetrain;
    Timer m_timer;

    public StopAtDistance(Drivetrain drivetrain) {
        m_rangeFinder = drivetrain.m_rangerFinder;
        m_boing = new Debouncer(.06);
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    public void execute() {
        if (m_timer.hasElapsed(1)) {

            m_boing.calculate(m_rangeFinder.getVoltage() <= .4);
            if (m_boing.calculate(m_rangeFinder.getVoltage() <= .4)) {
                m_drivetrain.arcadeDrive(0, 1);
            } else {
                m_drivetrain.arcadeDrive(1, 0);

            }
        }
    }

    public boolean isFinished() {
        return false;

    }

    public double GetDistanceinches() {
        return m_rangeFinder.getVoltage();

    }

    public void end(boolean interrupted) {
        m_drivetrain.arcadeDrive(1, 0);

    }

}
