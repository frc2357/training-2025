package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CAN_ID {

    public static final int FRONT_LEFT_DRIVE_MOTOR = 11;
    public static final int FRONT_LEFT_STEER_MOTOR = 12;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 13;
    public static final int FRONT_RIGHT_STEER_MOTOR = 14;

    public static final int BACK_LEFT_DRIVE_MOTOR = 15;
    public static final int BACK_LEFT_STEER_MOTOR = 16;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 17;
    public static final int BACK_RIGHT_STEER_MOTOR = 18;

    public static final int FRONT_LEFT_ENCODER = 19;
    public static final int FRONT_RIGHT_ENCODER = 20;
    public static final int BACK_LEFT_ENCODER = 21;
    public static final int BACK_RIGHT_ENCODER = 22;

    public static final int ELEVATOR_LEFT_MOTOR = 23;
    public static final int ELEVATOR_RIGHT_MOTOR = 24;

    public static final int CORAL_RUNNER_MOTOR = 29;
    public static final int ALGAE_KNOCKER_MOTOR = 33;

    public static final int LATERATOR_MOTOR = 28;
  }

  public static final class CONTROLLER {

    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final double CODRIVER_CONTROLLER_DEADBAND = 0.025;
    public static final double DRIVER_RUMBLE_INTENSITY = .5;
    public static final double CODRIVER_RUMBLE_INTENSITY = .5;
    public static final double DRIVER_RUMBLE_SECONDS = 2;
    public static final double CODRIVER_RUMBLE_SECONDS = 2;
    public static final double JOYSTICK_RAMP_EXPONENT = 1;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_RATE =
      RotationsPerSecond.of(1);
    public static final LinearVelocity MAX_SPEED =
      TunerConstants.kSpeedAt12Volts;

    public static final Dimensionless AXIS_MAX_ANGULAR_RATE = Units.Percent.of(
      25
    );
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(25);
  }

  public static class DIGITAL_INPUT {

    public static final int LATERATOR_HALL_EFFECT_ID = 9;
    public static final int CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID = 8;
    public static final int CORAL_RUNNER_BEAM_BREAK_INTAKE_ID = 7;
  }

  public static class CORAL_RUNNER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(50);
    public static final Time DEBOUNCE_TIME_SECONDS = Units.Milliseconds.of(10);

    public static final Dimensionless INTAKE_SPEED = Units.Percent.of(-50);
    public static final Dimensionless INTAKE_SPEED_SLOW = Units.Percent.of(-15);
    public static final Time INTAKE_SETTLE_TIME = Units.Seconds.of(.5);

    public static final Dimensionless SCORING_SPEED = Units.Percent.of(-75);
    public static final Time SCORING_RUN_TIME = Units.Seconds.of(.5);
  }

  public static class SENSOR_PERIODIC {

    public static final Time SENSOR_PERIODIC_TIME = Units.Milliseconds.of(5);
    public static final Time SENSOR_PERIODIC_OFFSET_TIME =
      Units.Milliseconds.of(3);
  }

  public static class ALGAE_KNOCKER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);
    public static final Dimensionless DE_ALGAE_SPEED = Units.Percent.of(50);
  }

  public static class ELEVATOR {

    public static final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);

    public static final SparkBaseConfig RIGHT_MOTOR_CONFIG =
      new SparkMaxConfig()
        .apply(LEFT_MOTOR_CONFIG)
        .follow(CAN_ID.ELEVATOR_LEFT_MOTOR, true);

    public static final double MOTOR_P = 1;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0.1;
    public static final double MOTOR_FF = 0;
    public static final double MAX_ACCEL = 0;
    public static final double MAX_VEL = 0;

    public static final double MOTOR_KS = 0;
    public static final double MOTOR_KG = 0.67;
    public static final double MOTOR_KV = 0.1;

    public static final Distance MAX_ALLOWED_ERROR = Units.Inches.of(.75);

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(25);
    public static final Time DEBOUNCE_TIME_SECONDS = Units.Milliseconds.of(15);

    public static final double GEAR_RATIO = (38.0 / 14.0) * 2.0;

    public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
    public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      HTD5_PULLEY_PITCH.times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

    public static final Dimensionless HOLD_PERCENT_OUTPUT = Units.Percent.of(7); //TODO: Find this value

    public static class SETPOINT {

      public static final Distance L1 = Units.Inches.of(1);
      public static final Distance L2 = Units.Inches.of(8.43);
      public static final Distance L3 = Units.Inches.of(24.189);
      public static final Distance L4 = Units.Inches.of(49.5);

      public static final Distance GROUND = Units.Feet.of(0);
      public static final Distance HOME = Units.Inches.of(2);

      public static final Distance INTAKE = Units.Inches.of(1.1);
      public static final Distance LOW_ALGAE = Units.Inches.of(0.5);
      public static final Distance HIGH_ALGAE = Units.Inches.of(13);
    }
  }

  public static class LATERATOR {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);

    public static final double MOTOR_P = 0.1;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0.01;
    public static final double MOTOR_FF = 0;
    public static final double MAX_ACCEL = 0;
    public static final double MAX_VEL = 0;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG =
      MOTOR_CONFIG.closedLoop
        .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_FF)
        .outputRange(-1, 1);

    public static final Angle MAX_ALLOWED_ERROR = Units.Rotations.of(.5);

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(50);
    public static final Time DEBOUNCE_TIME_SECONDS = Units.Milliseconds.of(10);

    public static final double GEAR_RATIO = 15;
    public static final Distance OUTPUT_PULLEY_PITCH_DIAMETER =
      Units.Millimeters.of(46.188);
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      OUTPUT_PULLEY_PITCH_DIAMETER.times(Math.PI);

    public static final Dimensionless ZERO_SPEED = Units.Percent.of(-25);

    public static class SETPOINT {

      public static final Distance INTAKE = Units.Inches.of(3);
      public static final Distance HOME = Units.Inches.of(1);
      public static final Distance MAX_SAFE_SCORING_EXTENSION = Units.Inches.of(
        -1
      );
      public static final Distance L1 = Units.Inches.of(-2);
      public static final Distance L2 = Units.Inches.of(-6.1);
      public static final Distance L3 = Units.Inches.of(-6.1);
      public static final Distance L4 = Units.Inches.of(-6.25);
      public static final Distance FULL_SCORING_EXTENSION = Units.Inches.of(
        -6.6
      );
    }
  }
}
