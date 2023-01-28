// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Lib.SwerveData;

/** Add your docs here. */
public class k {
    public static class DRIVETRAIN {
       public static double maxSpeed = 4.36; // m/s
       public static double maxAngularSpeed = Math.PI; // 1/2 Rotation/Sec
        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front to back 607mm, Side to side 640mm
            new Translation2d(0.3035, 0.320), // Front Left
            new Translation2d(0.3035, -0.320),
            new Translation2d(-0.3035, 0.0)
        );
    }
    public static class SWERVE {
        public static SwerveData SDBack = new SwerveData("back", 0, InvertType.None, 0, InvertType.None, 0, 0);
        public static SwerveData SDFrontLeft = new SwerveData("back", 0, InvertType.None, 0, InvertType.None, 0, 0);
        public static SwerveData SDFrontRight = new SwerveData("back", 0, InvertType.None, 0, InvertType.None, 0, 0);

        public static double driveKp = 0;
        public static double driveKi = 0;
        public static double driveKd = 0;

        public static double steerKp = 0;
        public static double steerKi = 0;
        public static double steerKd = 0;

        public static double steerMax_RadPS = 0;
        public static double steerMax_RadPSSq = 0;

        public static double steerSMFKs = 0;
        public static double steerSMFKv = 0;
        public static double steerSMFKa = 0;

        public static double driveSMFKs = 0;
        public static double driveSMFKv = 0;
        public static double driveSMFKa = 0;

        public static double driveDistanceCntsPMeter = 1;
        public static double steer_CntsPRad = 1;
        public static double driveRawVelocityToMPS = 1;
    }
    public static class ARM{
        public static int leftCANId = 20;
        public static int rightCANId = 21;
        public static double bicepP = 1;
    }
}
