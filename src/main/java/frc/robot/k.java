package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Lib.ArmData;
import frc.robot.Lib.SwerveData;

/** Add your docs here. */
public class k {
    public static class DRIVETRAIN {
       public static double maxSpeed = 4.36; // m/s
       public static double maxAngularSpeed = Math.PI; // 1/2 Rotation/Sec
        // public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        //     // Front to back 607mm, Side to side 640mm
        //     new Translation2d(0.3035, 0.320), // Front Left
        //     new Translation2d(0.3035, -0.320),
        //     new Translation2d(-0.3035, 0.0)
        // );
        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front to back 607mm, Side to side 640mm
            new Translation2d(0.3035, 0.0),   // Front        B
            new Translation2d(-0.3035, -0.320), // Back Right   FL
            new Translation2d(-0.3035, 0.320)    // Back Left   FR
            
        );
        public static double maxVoltage = 12.0;
        public static double rotKp = 0.01;
        public static double rotKi = 0.02;
        public static double rotKd = 0.0;
        public static double rotToleranceDeg = 1;
        public static double rotToleranceVel = 10; // Deg/sec
        public static double rotMaxOutput  = .5;
        public static double stickDeadband = 0.1;
        public static double speedScale = 1.0;
        public static double rotationScale = 1.0;
        public static double autoRotateOutScale = 1.0;
    }
    public static class SWERVE {
        public static SwerveData SDFront = new SwerveData("Front", 10, InvertType.None, 20, InvertType.InvertMotorOutput, 1, 193.3);
        public static SwerveData SDBackRight = new SwerveData("BRight",11, InvertType.InvertMotorOutput, 21, InvertType.InvertMotorOutput, 2, 167.7);
        public static SwerveData SDBackLeft = new SwerveData("BLeft",12, InvertType.None,22, InvertType.InvertMotorOutput, 3, 283);

        public static double steerKp = 3;
        public static double steerKi = 6;
        public static double steerKd = 0;

        public static double steerSMFKs = 0.0;
        public static double steerSMFKv = 0;
        public static double steerSMFKa = 0;

        public static double steerMax_RadPS = Math.PI;
        public static double steerMax_RadPSSq = Math.pow(steerMax_RadPS,2);
        public static double steer_CntsPRad = 5028.932;

        public static double driveKp = 0.5;
        public static double driveKi = 1.5;
        public static double driveKd = 0;

        public static double driveSMFKs = 0.0;
        public static double driveSMFKv = DRIVETRAIN.maxVoltage/DRIVETRAIN.maxSpeed;
        public static double driveSMFKa = 0;

        public static double driveDistanceCntsPMeter = 49907;
        public static double driveRawVelocityToMPS = 4990.68;
    }
    public static class OI {
        public static double stickAngleMax = 0.9;
    }
    public static class SHOULDER {
        public static int leftCANId = 30;
        public static int rightCANId = 31;
        public static double PID_P = 1;
        public static double PID_I = 1;
        public static double CntsPDeg = 1;
        public static double PercentOnTarget = 1;
        public static double TimeoutOnTarget = 2;
        public static ArmData shoulderData = new ArmData(0,0,0,0,0,0,0);
    }
    public static class ELBOW {
        public static int leftCANId = 32;
        public static int rightCANId = 33;
        public static double PID_P = 1;
        public static double PID_I = 1;
        public static double CntsPDeg = 1;
        public static double PercentOnTarget = 1;
        public static double TimeoutOnTarget = 2;
    }
    public static class INTAKE {
        public static int leftRotateCANId = 34;
        public static int rightSpinnerCANId = 35;
        public static double PID_P = 1;
        public static double PID_I = 1;
        public static double CntsPDeg = 1;
        public static double PercentOnTarget = 1;
        public static double TimeoutOnTarget = 2;
    }

}
