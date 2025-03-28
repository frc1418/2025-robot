package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.FieldSpaceOdometry;
import frc.robot.common.TargetSpaceOdometry;

public class DriveSubsystem extends SubsystemBase {
    
    private final MaxWheelModule frontLeftWheel = new MaxWheelModule(
        DrivetrainConstants.FRONT_LEFT_SPEED_ID,
        DrivetrainConstants.FRONT_LEFT_ANGLE_ID,
        DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET,
        DrivetrainConstants.FRONT_LEFT_P,
        DrivetrainConstants.FRONT_LEFT_D,
        DrivetrainConstants.FRONT_LEFT_KS,
        DrivetrainConstants.FRONT_LEFT_KV,
        DrivetrainConstants.FRONT_LEFT_KA
    );
      
    private final MaxWheelModule frontRightWheel = new MaxWheelModule(
        DrivetrainConstants.FRONT_RIGHT_SPEED_ID,
        DrivetrainConstants.FRONT_RIGHT_ANGLE_ID,
        DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET,
        DrivetrainConstants.FRONT_RIGHT_P,
        DrivetrainConstants.FRONT_RIGHT_D,
        DrivetrainConstants.FRONT_RIGHT_KS,
        DrivetrainConstants.FRONT_RIGHT_KV,
        DrivetrainConstants.FRONT_RIGHT_KA
    );

    private final MaxWheelModule backLeftWheel = new MaxWheelModule(
        DrivetrainConstants.BACK_LEFT_SPEED_ID,
        DrivetrainConstants.BACK_LEFT_ANGLE_ID,
        DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET,
        DrivetrainConstants.BACK_LEFT_P,
        DrivetrainConstants.BACK_LEFT_D,
        DrivetrainConstants.BACK_LEFT_KS,
        DrivetrainConstants.BACK_LEFT_KV,
        DrivetrainConstants.BACK_LEFT_KA
    );

    private final MaxWheelModule backRightWheel = new MaxWheelModule(
        DrivetrainConstants.BACK_RIGHT_SPEED_ID,
        DrivetrainConstants.BACK_RIGHT_ANGLE_ID,
        DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET,
        DrivetrainConstants.BACK_RIGHT_P,
        DrivetrainConstants.BACK_RIGHT_D,
        DrivetrainConstants.BACK_RIGHT_KS,
        DrivetrainConstants.BACK_RIGHT_KV,
        DrivetrainConstants.BACK_RIGHT_KA
    );

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry ntFrontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");
    private final NetworkTableEntry ntFrontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry ntBackLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry ntBackRightAngleEncoder = table.getEntry("backRightAngleEncoder");

    private final NetworkTableEntry ntFrontLeftSpeed = table.getEntry("frontLeftSpeed");
    private final NetworkTableEntry ntFrontRightSpeed = table.getEntry("frontRightSpeed");
    private final NetworkTableEntry ntBackLeftSpeed = table.getEntry("backLeftSpeed");
    private final NetworkTableEntry ntBackRightSpeed = table.getEntry("backRightSpeed");

    private final NetworkTableEntry ntFrontLeftPos = table.getEntry("frontLeftPos");
    private final NetworkTableEntry ntFrontRightPos = table.getEntry("frontRightPos");
    private final NetworkTableEntry ntBackLeftPos = table.getEntry("backLeftPos");
    private final NetworkTableEntry ntBackRightPos = table.getEntry("backRightPos");

    private final NetworkTableEntry ntIsFieldCentric = table.getEntry("isFieldCentric");
    private final NetworkTableEntry ntIsTempSlowMode = table.getEntry("isTempSlowMode");

    private final NetworkTableEntry ntHeading = table.getEntry("heading");
    private final NetworkTableEntry ntLockedRot = table.getEntry("lockedRot");
    private final NetworkTableEntry ntEstimatedRot = table.getEntry("estimatedRot");

    private final FieldSpaceOdometry fieldOdometry;
    private final TargetSpaceOdometry targetOdometry;
    private ClimbSubsystem climbSubsystem;

    private Optional<SwerveModulePosition[]> swerveModulePositions;

    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    //This PID controller is used to keep the robot facing the same direction when not rotating
    private PIDController rotationController = new PIDController(DriverConstants.baseCorrector, 0, 0); 

    private SlewRateLimiter limitX = new SlewRateLimiter(DriverConstants.maxAccel);
    private SlewRateLimiter limitY = new SlewRateLimiter(DriverConstants.maxAccel);

    private double lockedRot;
    private double previousRot = 0;

    private Optional<Alliance> ally;

    private boolean fieldCentric = true;
    private boolean limitDrive = true;
    private boolean tempSlowMode = false;
    private boolean permSlowMode = false;

    RobotConfig config;

    public DriveSubsystem(ClimbSubsystem climbSubsystem) {
        swerveModulePositions = getModulePositions();
        ally = DriverStation.getAlliance();
        fieldOdometry = new FieldSpaceOdometry(getFirstModulePositions());
        targetOdometry = new TargetSpaceOdometry(getFirstModulePositions(), fieldOdometry);
        this.climbSubsystem = climbSubsystem;
        resetLockRot();

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.out.println("Error loading config");
            e.printStackTrace();
        }

        AutoBuilder.configure(
            fieldOdometry::getPose, // Robot pose supplier
            fieldOdometry::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> chassisSpeedsDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              if (ally.isPresent()) {
                return ally.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public SwerveModulePosition[] getFirstModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftWheel.getPosition(),
            frontRightWheel.getPosition(),
            backLeftWheel.getPosition(),
            backRightWheel.getPosition()
        };
    }

    private Optional<SwerveModulePosition[]> getModulePositions() {
        SwerveModulePosition frontLeftPos = frontLeftWheel.getPosition();
        SwerveModulePosition frontRightPos = frontRightWheel.getPosition();
        SwerveModulePosition backLeftPos = backLeftWheel.getPosition();
        SwerveModulePosition backRightPos = backRightWheel.getPosition();
        
        if (frontLeftPos != null && frontRightPos != null 
        && backLeftPos != null && backRightPos != null) {
            return Optional.of(new SwerveModulePosition[] {
                frontLeftPos,
                frontRightPos,
                backLeftPos,
                backRightPos
            });
        }
        else {
            return null;
        }
    }

    public void drive(double x, double y, double rot) {
        if (tempSlowMode) {
            x *= 0.5;
            y *= 0.5;
            rot *= 0.5;
        }
        if (permSlowMode) {
            x *= 0.75;
            y *= 0.75;
            rot *= 0.75;
        }

        if (limitDrive) {
            x = limitX.calculate(x);
            y = limitY.calculate(y);
        }

        if (Math.abs(rot - previousRot) > DriverConstants.maxAngularAccel && Math.abs(rot) > Math.abs(previousRot)) {
            rot = previousRot+DriverConstants.maxAngularAccel*Math.signum(rot);
        }
        previousRot = rot;

        double xSpeed = x*DriverConstants.maxSpeedMetersPerSecond;
        double ySpeed = y*DriverConstants.maxSpeedMetersPerSecond;
        double rotSpeed = rot*DriverConstants.maxAngularSpeed;

        if (fieldOdometry.isRotJustCorrected()) {
            resetLockRot();
        }
        
        if (!ally.isPresent()) {
            ally = DriverStation.getAlliance();
        }

        if(rotSpeed == 0 && fieldOdometry.isCorrectRot() && Math.abs(fieldOdometry.getGyroHeading().getDegrees() - lockedRot) < 180 && DriverStation.isTeleopEnabled() && !climbSubsystem.isClimbing()) {
            if (Math.hypot(x, y) > 0.25) {
                rotationController.setP(Math.hypot(x,y)*DriverConstants.correctiveFactor);
            }
            else {
                rotationController.setP(DriverConstants.baseCorrector);
            }
            rotSpeed = rotationController.calculate(fieldOdometry.getGyroHeading().getDegrees(), lockedRot);
            if (Math.abs(rotSpeed) > DriverConstants.maxCorrectiveAngularSpeed) {
                rotSpeed = DriverConstants.maxCorrectiveAngularSpeed*Math.signum(rotSpeed);
            }
        }
        else {
            resetLockRot();
        }

        if (fieldCentric && fieldOdometry.isCorrectRot()) {
            if (ally.get() == Alliance.Red) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, new Rotation2d(fieldOdometry.getGyroHeading().getRadians()+Math.PI));
            }
            else {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, fieldOdometry.getGyroHeading());
            }
        }
        else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        chassisSpeedsDrive(speeds);
    }

    public void chassisSpeedsDrive(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        
        SwerveModuleState[] wheelStates = DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(wheelStates, DriverConstants.maxSpeedMetersPerSecond);
        moduleStatesDrive(wheelStates);
    }

    public void moduleStatesDrive(SwerveModuleState[] wheelStates) {
        frontLeftWheel.setDesiredState(wheelStates[0]);
        frontRightWheel.setDesiredState(wheelStates[1]);
        backLeftWheel.setDesiredState(wheelStates[2]);
        backRightWheel.setDesiredState(wheelStates[3]);
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public int getAprilTagNumber() {
      return fieldOdometry.getAprilTagNumber();
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public FieldSpaceOdometry getOdometry() {
        return fieldOdometry;
    }

    public TargetSpaceOdometry getTargetOdometry() {
        return targetOdometry;
    }

    public double getLockedRot() {
        return lockedRot;
    }

    public boolean getCorrectRot() {
        return fieldOdometry.isCorrectRot();
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return speeds;
    }

    public void resetLockRot() {
        lockedRot = fieldOdometry.getGyroHeading().getDegrees();
    }

    public Command resetPose(Pose2d pose) {
        return Commands.runOnce(() -> {
            fieldOdometry.resetPose(pose);
        });
    }

    public Command getRotError() {
        return Commands.runOnce(() -> {
            fieldOdometry.findRotError();
        });
    }

    public Command correctError() {
        return Commands.runOnce(() -> {
            fieldOdometry.correctError();
        });
    }

    public Command setTempSlowMode(boolean setting) {
        return Commands.runOnce(() -> {
            tempSlowMode = setting;
        });
    }

    public Command turtle() {
        return new RunCommand(() -> {
            frontLeftWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRightWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backLeftWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backRightWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }, this);
    }

    public Command toggleFastMode() {
        return Commands.runOnce(() -> {
            permSlowMode = !permSlowMode;
        });
    }

    public Command toggleLimitDrive() {
        return Commands.runOnce(() -> {
            limitDrive = !limitDrive;
            limitX.reset(0);
            limitY.reset(0);
        });
    }

    public Command toggleFieldCentric() {
        return Commands.runOnce(() -> {
            fieldCentric = !fieldCentric;
        });
    }

    public Command resetFieldCentric() {
        return Commands.runOnce(() -> {
            fieldOdometry.zeroHeading();
            resetLockRot();
            fieldOdometry.setCorrectRot(true);
        });
    }

    @Override
    public void periodic() {
        swerveModulePositions = getModulePositions();
        if (swerveModulePositions.isPresent()) {
            fieldOdometry.update(getModulePositions().get(), lockedRot);
            targetOdometry.update(getModulePositions().get());

            ntFrontLeftAngleEncoder.setDouble(swerveModulePositions.get()[0].angle.getRadians());
            ntFrontRightAngleEncoder.setDouble(swerveModulePositions.get()[1].angle.getRadians());
            ntBackLeftAngleEncoder.setDouble(swerveModulePositions.get()[2].angle.getRadians());
            ntBackRightAngleEncoder.setDouble(swerveModulePositions.get()[3].angle.getRadians());

            ntFrontLeftSpeed.setDouble(frontLeftWheel.getState().speedMetersPerSecond);
            ntFrontRightSpeed.setDouble(frontRightWheel.getState().speedMetersPerSecond);
            ntBackLeftSpeed.setDouble(backLeftWheel.getState().speedMetersPerSecond);
            ntBackRightSpeed.setDouble(backRightWheel.getState().speedMetersPerSecond);

            ntFrontLeftPos.setDouble(swerveModulePositions.get()[0].distanceMeters);
            ntFrontRightPos.setDouble(swerveModulePositions.get()[1].distanceMeters);
            ntBackLeftPos.setDouble(swerveModulePositions.get()[2].distanceMeters);
            ntBackRightPos.setDouble(swerveModulePositions.get()[3].distanceMeters);
        }
        else {
            fieldOdometry.update(null, lockedRot);
            targetOdometry.update(null);
        }

        ntIsFieldCentric.setBoolean(fieldCentric);
        ntIsTempSlowMode.setBoolean(tempSlowMode);

        ntHeading.setDouble(fieldOdometry.getGyroHeading().getDegrees());
        ntLockedRot.setDouble(lockedRot);
        ntEstimatedRot.setDouble(fieldOdometry.getEstimatedRot().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}