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
        public static double maxVoltage = 12.0;
    }
    public static class SWERVE {
        public static SwerveData SDBack = new SwerveData("Back", 10, InvertType.None, 20, InvertType.InvertMotorOutput, 1, 14);
        public static SwerveData SDFrontLeft = new SwerveData("FLeft",11, InvertType.None, 21, InvertType.InvertMotorOutput, 2, 163.5);
        public static SwerveData SDFrontRight = new SwerveData("FRight",12, InvertType.InvertMotorOutput,22, InvertType.InvertMotorOutput, 3, 282);



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

        public static double driveSMFKs = 0.025;
        public static double driveSMFKv = 3.2;
        public static double driveSMFKa = 0;

        public static double driveDistanceCntsPMeter = 1;
        public static double driveRawVelocityToMPS = 4990.68;
    }
    public static class ARM{
        public static int leftShoulderCANId = 30;
        public static int rightShoulderCANId = 31;
        public static int leftElbowCANId = 32;
        public static int rightElbowCANId = 33;
        public static int leftIntakeRotateCANId = 34;
        public static int rightIntakeSpinnerCANId = 35;
        public static double shoulderP = 1;
        public static double shoulderCntsPDeg = 1;
        public static double elbowCntsPDeg = 1;
        public static double intakeCntsPDeg = 1;

        
    }

}
