// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Telemetry logger = new Telemetry(
    Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond)
  );

  public RobotContainer() {
    Robot.swerve.registerTelemetry(logger::telemeterize);

    Robot.swerve.setDefaultCommand(Robot.defaultDrive);
    System.out.println("default drive set");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
