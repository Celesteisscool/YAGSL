// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static HolonomicPathFollowerConfig swervePathFollowerConfig  = new HolonomicPathFollowerConfig(
    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    1.3716, // Max module speed, in m/s
    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    new ReplanningConfig() // Default path replanning config. See the API for the options here
  );
}
