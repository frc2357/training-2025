package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase {

  private SparkMax m_left_Motor;
  private SparkMax m_right_Motor;

  private ProfiledPIDController m_PIDController;
  private RelativeEncoder m_encoder;

  private MutAngle m_targetRotations = Rotations.mutable(Double.NaN);
  private MutAngle m_motorAngle = Rotations.mutable(0);
  private MutVoltage m_motorVoltage = Volts.mutable(0);

  private ElevatorFeedforward MOTOR_FF = new ElevatorFeedforward(
    Constants.ELEVATOR.MOTOR_KS,
    Constants.ELEVATOR.MOTOR_KG,
    Constants.ELEVATOR.MOTOR_KV
  );

  public Elevator() {
    m_left_Motor = new SparkMax(
      Constants.CAN_ID.ELEVATOR_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_left_Motor.configure(
      Constants.ELEVATOR.LEFT_MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_right_Motor = new SparkMax(
      Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR,
      MotorType.kBrushless
    );
    m_right_Motor.configure(
      Constants.ELEVATOR.RIGHT_MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = new ProfiledPIDController(
      ELEVATOR.MOTOR_P,
      ELEVATOR.MOTOR_I,
      ELEVATOR.MOTOR_D,
      new TrapezoidProfile.Constraints(ELEVATOR.MAX_VEL, ELEVATOR.MAX_ACCEL)
    );

    m_encoder = m_left_Motor.getEncoder();
  }

  public void setSpeed(Dimensionless speed) {
    m_targetRotations.mut_replace(Double.NaN, Rotations);
    m_left_Motor.set(speed.in(Units.Value));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    m_targetRotations.mut_replace(Double.NaN, Rotations);
    m_left_Motor.set(axisSpeed.times(ELEVATOR.AXIS_MAX_SPEED).in(Percent));
  }

  public void updateMotorPIDs() {
    m_left_Motor.setVoltage(
      m_PIDController.calculate(m_encoder.getPosition()) +
      MOTOR_FF.calculate(m_PIDController.getSetpoint().velocity)
    );
  }

  public void stop() {
    m_targetRotations.mut_replace(Double.NaN, Rotations);
    m_left_Motor.set(0);
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations.mut_replace(targetRotations);
    m_PIDController.setGoal(m_targetRotations.in(Rotations));
  }

  private Angle getRotations() {
    return m_motorAngle.mut_replace(m_encoder.getPosition(), Rotations);
  }

  public boolean isAtTarget() {
    return getDistance()
      .isNear(
        rotationsToDistance(m_targetRotations),
        Constants.ELEVATOR.MAX_ALLOWED_ERROR
      );
  }

  public void setTargetDistance(Distance targetDistance) {
    setTargetRotations(distanceToRotations(targetDistance));
  }

  public Distance getDistance() {
    return rotationsToDistance(getRotations());
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  private Distance rotationsToDistance(Angle rotations) {
    return (
      ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        rotations.div(ELEVATOR.GEAR_RATIO).in(Rotations)
      )
    );
  }

  private Angle distanceToRotations(Distance distance) {
    return Rotations.of(
      distance
        .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(ELEVATOR.GEAR_RATIO)
        .magnitude()
    );
  }

  public Voltage motorVoltage() {
    return m_motorVoltage.mut_replace(m_left_Motor.getBusVoltage(), Volts);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rotations", getRotations().in(Rotations));
    SmartDashboard.putNumber("Distance", getDistance().in(Inches));
  }
}
