// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public final static class DrivetrainConstants{

    public static final double wheelPos = 0.33655;

    public static final double DRIFT_DEADBAND = 0.15;
    public static final double ROTATION_DEADBAND = 0.03;

    public static final int FRONT_LEFT_ANGLE_ID = 3;
    public static final int FRONT_LEFT_SPEED_ID = 4;
    public static final Translation2d FRONT_LEFT_LOC = new Translation2d(wheelPos, wheelPos);
    public static final double FRONT_LEFT_ENCODER_OFFSET = 3.43+Math.PI;
    public static final double FRONT_LEFT_P = 0.00040465;
    public static final double FRONT_LEFT_D = 0;
    public static final double FRONT_LEFT_KS = 0.21461;
    public static final double FRONT_LEFT_KV = 2.8012;
    public static final double FRONT_LEFT_KA = 0.7612;

    public static final int FRONT_RIGHT_ANGLE_ID = 5;
    public static final int FRONT_RIGHT_SPEED_ID = 6;
    public static final Translation2d FRONT_RIGHT_LOC = new Translation2d(wheelPos, -wheelPos);
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.7968+Math.PI;
    public static final double FRONT_RIGHT_P = 0.00091514;
    public static final double FRONT_RIGHT_D = 0;
    public static final double FRONT_RIGHT_KS = 0.34839;
    public static final double FRONT_RIGHT_KV = 2.7542;
    public static final double FRONT_RIGHT_KA = 0.55833;
    
    public static final int BACK_LEFT_ANGLE_ID = 1;
    public static final int BACK_LEFT_SPEED_ID = 2;
    public static final Translation2d BACK_LEFT_LOC = new Translation2d(-wheelPos, wheelPos);
    public static final double BACK_LEFT_ENCODER_OFFSET = 2.1945+Math.PI;
    public static final double BACK_LEFT_P = 0.0015266;
    public static final double BACK_LEFT_D = 0;
    public static final double BACK_LEFT_KS = 0.19566;
    public static final double BACK_LEFT_KV = 2.6945;
    public static final double BACK_LEFT_KA = 0.57784;

    public static final int BACK_RIGHT_ANGLE_ID = 7;
    public static final int BACK_RIGHT_SPEED_ID = 8;
    public static final Translation2d BACK_RIGHT_LOC = new Translation2d(-wheelPos, -wheelPos);
    public static final double BACK_RIGHT_ENCODER_OFFSET = 1.042+Math.PI;
    public static final double BACK_RIGHT_P = 0.0010266;
    public static final double BACK_RIGHT_D = 0;
    public static final double BACK_RIGHT_KS = 0.23609;
    public static final double BACK_RIGHT_KV = 2.7405;
    public static final double BACK_RIGHT_KA = 0.56268;
    
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT_LOC,
      FRONT_RIGHT_LOC,
      BACK_LEFT_LOC,
      BACK_RIGHT_LOC
    );
  }

  public final static class ElevatorConstants {
    public final static int ELEVATOR_MOTOR_1_ID = 9;
    public final static int ELEVATOR_MOTOR_2_ID = 10;

    public final static double ENCODER_SCALAR = 4.43;
    public final static double ELEVATOR_OFFSET = 0.037;

    public final static double kV = 0.12;
    public final static double kG = 0.175;
    public final static double kCoral = 0.025;

    public final static double kP = 0.65;
    public final static double kD = 0.25;
  }

  public final static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 11;
  }

  public final static class PivotConstants {
    public static final int TALON_MOTOR_ID = 1;
    public final static double ENCODER_SCALAR = -0.97;

    public static final double PIVOT_OFFSET = -0.578888889; //36 degrees up 22.2 percent elevator to intake
    // kG plus kConstant must be no more than 0.04 to keep falcon happy
    public static final double kCoral = 0.01;
    public static final double kG = 0.02;
    public static final double kConstant = 0.025;
    public static final double kV = 0.1;

    public final static double kP = 0.001;
    public final static double kI = 0;
    public final static double kD = 0.00025;
  }
  public final static class PneumaticsConstants {
    public final static int PNEUMATICS_HUB_ID = 21;
    
    public final static int CLIMB_SOLENOID_REVERSE = 0;
    public final static int CLIMB_SOLENOID_FORWARD = 1;  

    public final static int ATTACH_SOLENOID_REVERSE = 2;
    public final static int ATTACH_SOLENOID_FORWARD = 3;
  }

  public final static class DriverConstants {
    public final static double maxAccel = 1.5;
    public final static double maxSpeedMetersPerSecond = 4.8;
    public final static double maxAngularAccel = 0.075;
    public final static double maxAngularSpeed = 2*Math.PI;
    public final static double maxCorrectiveAngularSpeed = Math.PI;
    public final static double correctiveFactor = 0.16;
    public final static double baseCorrector = 0.04;
  }

  public final static class WheelConstants {
    /*
     * Conversion factor from motor rotations to meters
     * 1.5 is the wheel radius in inches
     * 0.0254 converts the radius to meters
     * 2 Pi converts to radius to circumference
     * 5.08 is the gear ratio between the motor and the wheel
     */
    public final static double ROTATIONS_TO_METERS = 1.5*0.0254*2*Math.PI/5.08; 
    public final static double TURNING_FACTOR = 2 * Math.PI;
  }
}
