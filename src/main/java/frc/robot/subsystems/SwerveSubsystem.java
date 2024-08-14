// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try {
      SwerveDrive  swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}