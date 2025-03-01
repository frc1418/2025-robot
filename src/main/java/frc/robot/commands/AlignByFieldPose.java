package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.common.FieldSpaceOdometry;
import frc.robot.subsystems.DriveSubsystem;

public class AlignByFieldPose extends Command {
  DriveSubsystem swerveDrive;
  FieldSpaceOdometry odometry;

  double targetX;
  double targetY;
  double targetRot;
  double sideOffset;
  double backOffset;
  double rotOffset;

  double kV;
  double initialSpeedP;
  double initialRotP = 0.01;

  int aligningTag = -1;

  boolean startedFieldCentric;
  boolean hasTargetPos = false;

  PIDController speedController;
  PIDController speedRotController;

  public enum AlignDirection {
    LEFT,
    RIGHT
  }
  
  private AlignDirection side;

  public AlignByFieldPose(DriveSubsystem swerveDrive, double sideOffset, double backOffset, double rotOffset, double P, double I, double D, double kV, double maxAccel, AlignDirection side) {
      this.swerveDrive = swerveDrive;
      this.odometry = swerveDrive.getOdometry();
      this.initialSpeedP = P;
      this.sideOffset = sideOffset;
      this.backOffset = backOffset;
      this.rotOffset = rotOffset;
      this.kV = kV;
      this.side = side;
      
      speedController = new PIDController(P, I, D);
      speedController.setTolerance(0.05);
      speedRotController = new PIDController(0.01, 0, 0);
      speedRotController.enableContinuousInput(-180, 180);

      addRequirements(swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      this.startedFieldCentric = swerveDrive.getFieldCentric();
      this.swerveDrive.setFieldCentric(false);
      this.aligningTag = odometry.getAprilTagNumber();

      if (odometry.getClosestAprilTagPose().isPresent()) {
        if (side == AlignDirection.LEFT) {
          sideOffset *= -1 - DriverConstants.armOffset;
        }

        double tagRotation = odometry.getClosestAprilTagPose().get().getRotation().toRotation2d().getDegrees();
        double offsetX = backOffset * Math.cos(Math.toRadians(tagRotation)) - sideOffset * Math.sin(Math.toRadians(tagRotation));
        double offsetY = backOffset * Math.sin(Math.toRadians(tagRotation)) + sideOffset * Math.cos(Math.toRadians(tagRotation));

        targetX = odometry.getClosestAprilTagPose().get().getX()+offsetX;
        targetY = odometry.getClosestAprilTagPose().get().getY()+offsetY;
        targetRot = tagRotation+180+rotOffset;
        System.out.println("targetX: " + targetX);
        System.out.println("targetY: " + targetY);
        System.out.println("targetRot: " + targetRot);
        hasTargetPos = true;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasTargetPos) {
      if (odometry.getClosestAprilTagPose().isPresent()) {
        if (side == AlignDirection.LEFT) {
          sideOffset *= -1 - DriverConstants.armOffset;
        }

        double tagRotation = odometry.getClosestAprilTagPose().get().getRotation().toRotation2d().getDegrees();
        double offsetX = backOffset * Math.cos(Math.toRadians(tagRotation)) - sideOffset * Math.sin(Math.toRadians(tagRotation));
        double offsetY = backOffset * Math.sin(Math.toRadians(tagRotation)) + sideOffset * Math.cos(Math.toRadians(tagRotation));

        targetX = odometry.getClosestAprilTagPose().get().getX()+offsetX;
        targetY = odometry.getClosestAprilTagPose().get().getY()+offsetY;
        targetRot = tagRotation+180+rotOffset;
        System.out.println("targetX: " + targetX);
        System.out.println("targetY: " + targetY);
        System.out.println("targetRot: " + targetRot);
        hasTargetPos = true;
      }
    }
    else if (swerveDrive.getCorrectRot()) {
      Pose2d targetPose;
      targetPose = new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(targetRot));
      
      double x;
      double y;
      double rot;
      double speed;

      double dx = targetPose.getX() - odometry.getPose().getX();
      double dy =  targetPose.getY() - odometry.getPose().getY();

      double distance = Math.hypot(dx, dy);
      if (distance < 2) {
        double angleToTarget = Math.atan2(dy, dx) * 180 / Math.PI;
        double deltaRot = Math.abs(targetRot - odometry.getGyroHeading().getDegrees());
  
        if (deltaRot < 5) {
            speedRotController.setP(initialRotP/((deltaRot+1)/6));
        }
        rot = speedRotController.calculate(odometry.getGyroHeading().getDegrees(), targetRot);
  
        if (distance < 0.9) {
            speedController.setP(initialSpeedP/(distance+0.1));
        }
        speed = speedController.calculate(0, distance)+kV;
  
        Rotation2d direction = Rotation2d.fromDegrees(angleToTarget - odometry.getGyroHeading().getDegrees());
  
        if (!speedController.atSetpoint()) {
            x = (direction.getCos() * speed);
            y = (direction.getSin() * speed);
        }
        else {
            x = 0;
            y = 0;
        }

        swerveDrive.drive(x, y, rot);     
      }
      else {
        System.out.println("TRYING TO GO A DISTANCE OF: " + distance);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END");
    speedController.reset();
    speedController.setP(initialSpeedP);
    speedRotController.reset();
    speedRotController.setP(initialRotP);
    this.swerveDrive.setFieldCentric(startedFieldCentric);
    this.aligningTag = -1;
    hasTargetPos = false;
    swerveDrive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}