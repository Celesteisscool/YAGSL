// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;

public class DriveSubsystem extends SubsystemBase {


  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public SwerveDrive swerveDrive = swerveSubsystem.swerveDrive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
			this::getPose2d,
			this::resetPose2d,
			this::getRobotRelativeSpeeds,
			this::driveRobotRelative,
			Constants.swervePathFollowerConfig,
			() -> {
				// Boolean supplier that controls when the path will be mirrored for the red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
				  return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			this
		);
  }

  public Pose2d getPose2d() {
    return swerveDrive.getPose();
  }

  public void resetPose2d(Pose2d pose2d) {
    swerveDrive.resetOdometry(pose2d);
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    swerveDrive.drive(chassisSpeeds);
  }

  public void driveCommand(Joystick driverController) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
		driverController.getRawAxis(0), 
		driverController.getRawAxis(1), 
		driverController.getRawAxis(2));
	  swerveDrive.driveFieldOriented(chassisSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
