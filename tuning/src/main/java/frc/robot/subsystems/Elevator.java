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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
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

  private Angle m_targetRotations = Rotations.of(Double.NaN);

  private double MOTOR_P = 0;
  private double MOTOR_I = 0;
  private double MOTOR_D = 0;
  private double MAX_ACCEL = 0;
  private double MAX_VEL = 0;
  private double MOTOR_KS = 0;
  private double MOTOR_KG = 0;
  private double MOTOR_KV = 0;

  private ElevatorFeedforward MOTOR_FF = new ElevatorFeedforward(
    MOTOR_KS,
    MOTOR_KG,
    MOTOR_KV
  );

  public Elevator() {
    SmartDashboard.putNumber("P", MOTOR_P);
    SmartDashboard.putNumber("I", MOTOR_I);
    SmartDashboard.putNumber("D", MOTOR_D);
    SmartDashboard.putNumber("KS", MOTOR_KS);
    SmartDashboard.putNumber("KG", MOTOR_KG);
    SmartDashboard.putNumber("KV", MOTOR_KV);
    SmartDashboard.putNumber("MaxVel", MAX_VEL);
    SmartDashboard.putNumber("MaxAcc", MAX_ACCEL);
    SmartDashboard.putNumber("Target Distance", 0);
    SmartDashboard.putNumber("Voltage", 0);
    SmartDashboard.putNumber("PID_Setpoint_Position", 0);
    SmartDashboard.putNumber("PID_Setpoint_Velocity", 0);

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
    setZero();
  }

  public void setSpeed(Dimensionless speed) {
    m_targetRotations = Rotations.of(Double.NaN);
    m_left_Motor.set(speed.in(Percent));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    m_targetRotations = Rotations.of(Double.NaN);
    m_left_Motor.set(axisSpeed.times(ELEVATOR.AXIS_MAX_SPEED).in(Percent));
  }

  public void updateMotor() {
    m_left_Motor.setVoltage(
      m_PIDController.calculate(m_encoder.getPosition()) +
      MOTOR_FF.calculate(m_PIDController.getSetpoint().velocity)
    );
  }

  public void stop() {
    m_targetRotations = Rotations.of(Double.NaN);
    m_left_Motor.set(0);
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setGoal(m_targetRotations.in(Rotations));
  }

  private Angle getRotations() {
    return Rotations.of(m_encoder.getPosition());
  }

  public void setTargetDistance(Distance targetDistance) {
    setTargetRotations(distanceToRotations(targetDistance));
  }

  public Distance getDistance() {
    return rotationsToDistance(getRotations());
  }

  public boolean isAtTargetDistance() {
    return getDistance()
      .isNear(
        rotationsToDistance(m_targetRotations),
        Constants.ELEVATOR.MAX_ALLOWED_ERROR.in(Inches)
      );
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  private Angle distanceToRotations(Distance distance) {
    return Rotations.of(
      distance
        .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(ELEVATOR.GEAR_RATIO)
        .magnitude()
    );
  }

  private Distance rotationsToDistance(Angle rotations) {
    return (
      ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        rotations.div(ELEVATOR.GEAR_RATIO).in(Rotations)
      )
    );
  }

  public Voltage motorVoltage() {
    return Volts.of(m_left_Motor.getBusVoltage());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rotations", getRotations().in(Rotations));
    SmartDashboard.putNumber("Distance", getDistance().in(Inches));
    SmartDashboard.putNumber("Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Voltage", motorVoltage().in(Volts));
    SmartDashboard.putNumber(
      "PID_Setpoint_Position",
      m_PIDController.getSetpoint().position
    );
    SmartDashboard.putNumber(
      "PID_Setpoint_Velocity",
      m_PIDController.getSetpoint().velocity
    );

    SmartDashboard.putBoolean("Is At Target", isAtTargetDistance());

    Distance target_Distance = Inches.of(
      SmartDashboard.getNumber("Target Distance", 0)
    );

    MOTOR_P = SmartDashboard.getNumber("P", MOTOR_P);
    MOTOR_I = SmartDashboard.getNumber("I", MOTOR_I);
    MOTOR_D = SmartDashboard.getNumber("D", MOTOR_D);
    MOTOR_KS = SmartDashboard.getNumber("KS", MOTOR_KS);
    MOTOR_KG = SmartDashboard.getNumber("KG", MOTOR_KG);
    MOTOR_KV = SmartDashboard.getNumber("KV", MOTOR_KV);
    MAX_VEL = SmartDashboard.getNumber("MaxVel", MAX_VEL);
    MAX_ACCEL = SmartDashboard.getNumber("MaxAcc", MAX_ACCEL);
    MOTOR_FF = new ElevatorFeedforward(MOTOR_KS, MOTOR_KG, MOTOR_KV);

    m_PIDController.setPID(MOTOR_P, MOTOR_I, MOTOR_D);
    m_PIDController.setConstraints(
      new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
    );

    setTargetDistance(target_Distance);
    updateMotor();
  }
}
