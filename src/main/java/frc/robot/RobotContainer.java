// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);

  public DriveSubsystem driveSubsystem = new DriveSubsystem();

  	/** The container for the robot. Contains subsystems, OI devices, and commands. */
  	public RobotContainer() {}
  
  public Command driveCommand() {
    return driveSubsystem.driveCommand(m_driverController);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }

  public void updateSim() {
    
    driveSubsystem.swerveDrive.updateOdometry();
    driveSubsystem.swerveDrive.synchronizeModuleEncoders();
    driveSubsystem.swerveDrive.setGyro(new Rotation3d(0, 0, 0));
  }
}
