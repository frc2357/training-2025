// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRunner;

public class Robot extends TimedRobot {

  public static CoralRunner coralRunner;
  public static CommandSwerveDrivetrain swerve;

  public static DriverControls driverControls;

  public static Command defaultDrive;
  private Command m_autonomousCommand;

  private final Telemetry logger = new Telemetry(
      Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond));

  public Robot() {
    swerve = TunerConstants.createDrivetrain();
    coralRunner = new CoralRunner();
    driverControls = new DriverControls(
        new CommandXboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT),
        Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);

    defaultDrive = new DefaultDrive();

    Robot.swerve.registerTelemetry(logger::telemeterize);
    Robot.swerve.setDefaultCommand(Robot.defaultDrive);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
