package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.FieldSpaceOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class AlignByFieldPose extends Command {
  DriveSubsystem swerveDrive;
  LedSubsystem leds;
  FieldSpaceOdometry odometry;

  double targetX;
  double targetY;
  double targetRot;
  double sideOffset;
  double backOffset;
  double rotOffset;
  double deltaSide = 0;
  double deltaBack = 0;

  double kV;
  double initialSpeedP;
  double initialRotP = 0.01;

  int aligningTag = -1;

  boolean startedFieldCentric;
  boolean hasTargetPos = false;

  PIDController speedController;
  PIDController speedRotController;
  
  public AlignByFieldPose(DriveSubsystem swerveDrive, LedSubsystem leds, double sideOffset, double backOffset, double rotOffset, double P, double I, double D, double kV, double maxAccel) {
    DataLogManager.start();  
    this.swerveDrive = swerveDrive;
    this.leds = leds;
    this.odometry = swerveDrive.getOdometry();
    this.initialSpeedP = P;
    this.sideOffset = sideOffset;
    this.backOffset = backOffset;
    this.rotOffset = rotOffset;
    this.kV = kV;
    
    speedController = new PIDController(P, I, D);
    speedController.setTolerance(0.006);
    speedRotController = new PIDController(0.01, 0, 0);
    speedRotController.enableContinuousInput(-180, 180);

    addRequirements(swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Delta Side: " + deltaSide);
    System.out.println("Delta Back: " + deltaBack);
      this.startedFieldCentric = swerveDrive.getFieldCentric();
      this.swerveDrive.setFieldCentric(false);
      this.aligningTag = odometry.getAprilTagNumber();

      if (odometry.getClosestAprilTagPose().isPresent()) {
        try {
          double newBackOffset = backOffset + deltaBack;
          double newSideOffset = sideOffset + deltaSide;     
  
          double tagRotation = odometry.getClosestAprilTagPose().get().getRotation().toRotation2d().getDegrees();
  
          double offsetX = newBackOffset * Math.cos(Math.toRadians(tagRotation)) - newSideOffset * Math.sin(Math.toRadians(tagRotation));
          double offsetY = newBackOffset * Math.sin(Math.toRadians(tagRotation)) + newSideOffset * Math.cos(Math.toRadians(tagRotation));
  
          targetX = odometry.getClosestAprilTagPose().get().getX()+offsetX;
          targetY = odometry.getClosestAprilTagPose().get().getY()+offsetY;
          targetRot = tagRotation+180+rotOffset;
          hasTargetPos = true;
          leds.setAligningColor();
        } catch (Exception e) {
          System.out.println("Error: " + e.getStackTrace());
        }
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasTargetPos) {
      System.out.println("Delta Side: " + deltaSide);
      System.out.println("Delta Back: " + deltaBack);
      if (odometry.getClosestAprilTagPose().isPresent()) {
        try {
          double newBackOffset = backOffset + deltaBack;
          double newSideOffset = sideOffset + deltaSide;        
  
          double tagRotation = odometry.getClosestAprilTagPose().get().getRotation().toRotation2d().getDegrees();
  
          double offsetX = newBackOffset * Math.cos(Math.toRadians(tagRotation)) - newSideOffset * Math.sin(Math.toRadians(tagRotation));
          double offsetY = newBackOffset * Math.sin(Math.toRadians(tagRotation)) + newSideOffset * Math.cos(Math.toRadians(tagRotation));
  
          targetX = odometry.getClosestAprilTagPose().get().getX()+offsetX;
          targetY = odometry.getClosestAprilTagPose().get().getY()+offsetY;
          targetRot = tagRotation+180+rotOffset;
          hasTargetPos = true;
          leds.setAligningColor();
        } catch (Exception e) {
          System.out.println("Error: " + e.getStackTrace());
        }
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
      if (distance < 3) {
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
            leds.setAlignedColor();
        }

        swerveDrive.drive(x, y, rot);     
      }
      else {
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
    leds.setAllianceColor();
  }

  public void adjustBackOffset(double delta, String alignmentType) {
    deltaBack += delta;
    hasTargetPos = false;
    DataLogManager.log(alignmentType + " backOffset:  " + deltaBack);
  }

  public void adjustSideOffset(double delta, String alignmentType) {
    deltaSide += delta;
    hasTargetPos = false;
    DataLogManager.log(alignmentType + " sideOffset:  " + deltaSide);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}