// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }

  public static class CAN_IDS {

    public static final int CORAL_RUNNER_MOTOR = 29;
  }

  public static class DIGITAL_INPUT {

    public static final int CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID = 8;
    public static final int CORAL_RUNNER_BEAM_BREAK_INTAKE_ID = 7;
  }

  public static class CORAL_RUNNER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(.12)
      .smartCurrentLimit(40, 40);
    public static final double AXIS_MAX_SPEED = .2;
    public static final double DEBOUNCE_TIME_SECONDS = .03;
  }
}
