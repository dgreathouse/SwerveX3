// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.lib.SwerveData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class k {
    public static class SWERVE {
        public static SwerveData LFData = new SwerveData("LF", 14, InvertType.None, 10, InvertType.InvertMotorOutput, 3, 41);
        public static SwerveData RFData = new SwerveData("RF", 16, InvertType.InvertMotorOutput, 12, InvertType.InvertMotorOutput, 5, 230);
        public static SwerveData BData = new SwerveData("B", 15, InvertType.InvertMotorOutput, 11, InvertType.InvertMotorOutput, 4, 153);
    }
    public static class CHASSIS {
        // See Constants spreadsheet for calculation of values
        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.1257, 0.235), // Front Left
            new Translation2d(0.1257, -0.235),
            new Translation2d(-0.27135, 0.0)
        );
    }
}
