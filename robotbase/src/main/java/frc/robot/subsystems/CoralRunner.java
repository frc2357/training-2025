package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralRunner extends SubsystemBase {

  private SparkMax motor;

  private DigitalInput BeamBreakSensor_Intake;
  private DigitalInput BeamBreakSensor_Outtake;

  private Debouncer debouncer_Intake;
  private Debouncer debouncer_Outtake;

  public CoralRunner() {
    motor = new SparkMax(
        Constants.CAN_ID.CORAL_RUNNER_MOTOR,
        MotorType.kBrushless);
    motor.configure(
        Constants.CORAL_RUNNER.MOTOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    BeamBreakSensor_Intake = new DigitalInput(
        Constants.DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_INTAKE_ID);
    BeamBreakSensor_Outtake = new DigitalInput(
        Constants.DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID);

    debouncer_Intake = new Debouncer(
        Constants.CORAL_RUNNER.DEBOUNCE_TIME_SECONDS,
        DebounceType.kBoth);
    debouncer_Outtake = new Debouncer(
        Constants.CORAL_RUNNER.DEBOUNCE_TIME_SECONDS,
        DebounceType.kBoth);
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless speed = axisSpeed.times(Constants.CORAL_RUNNER.AXIS_MAX_SPEED);

    motor.set(speed.in(Percent));
  }

  public void setSpeed(Dimensionless speed) {
    motor.set(speed.in(Percent));
  }

  public boolean isIntakeBeamBroken() {
    return debouncer_Intake.calculate(BeamBreakSensor_Intake.get());
  }

  public boolean isOuttakeBeamBroken() {
    return debouncer_Outtake.calculate(BeamBreakSensor_Outtake.get());
  }

  public void stop() {
    motor.stopMotor();
  }
}
