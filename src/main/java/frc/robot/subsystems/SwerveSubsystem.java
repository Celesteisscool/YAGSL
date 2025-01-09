// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public double maximumSpeed = Units.feetToMeters(4.5);
  public File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  // public SwerveDrive swerveDrive;
  public SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      if (RobotBase.isSimulation()) {
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
      }
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }
    catch (IOException e) {} // This prevents compiler errors :(
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}

