package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralRunner extends SubsystemBase {
    private SparkMax m_motor;

    private DigitalInput m_BeamBreakSensor_Intake;
    private DigitalInput m_BeamBreakSensor_Outtake;

    private Debouncer m_debouncer_Intake;
    private Debouncer m_debouncer_Outtake;

    public CoralRunner() {
        m_motor = new SparkMax(Constants.CAN_IDS.CORAL_RUNNER_MOTOR, MotorType.kBrushless);
        m_motor.configure(Constants.CORAL_RUNNER.MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_BeamBreakSensor_Intake = new DigitalInput(Constants.DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_INTAKE_ID);
        m_BeamBreakSensor_Outtake = new DigitalInput(Constants.DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID);

        m_debouncer_Intake = new Debouncer(Constants.CORAL_RUNNER.DEBOUNCE_TIME_SECONDS, DebounceType.kBoth);
        m_debouncer_Outtake = new Debouncer(Constants.CORAL_RUNNER.DEBOUNCE_TIME_SECONDS, DebounceType.kBoth);
    }

    public void setAxisSpeed(double axisSpeed) {
        double speed = axisSpeed * Constants.CORAL_RUNNER.AXIS_MAX_SPEED;
        m_motor.set(speed);
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public boolean isIntakeBeamBroken() {
        return m_debouncer_Intake.calculate(m_BeamBreakSensor_Intake.get());
    }

    public boolean isOuttakeBeamBroken() {
        return m_debouncer_Outtake.calculate(m_BeamBreakSensor_Outtake.get());
    }

    public void stop() {
        m_motor.stopMotor();
    }

}
