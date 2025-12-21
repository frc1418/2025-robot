// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.MAXSwerveModule;

public class MaxWheelModule extends SubsystemBase {

  private SparkMax speedMotor;
  private SparkMax angleMotor;


  private RelativeEncoder speedEncoder;
  private AbsoluteEncoder angleEncoder;

  private SparkClosedLoopController speedController;
  private SparkClosedLoopController angleController;

  private SimpleMotorFeedforward speedFeedforward;


  public MaxWheelModule(int speedMotorID, int angleMotorID, double chassisAngularOffset, double p,
      double d, double ks, double kv, double ka) {
    this.speedMotor = new SparkMax(speedMotorID, MotorType.kBrushless);

    SparkBaseConfig speedConfig = MAXSwerveModule.SpeedConfig;
    speedConfig.closedLoop.pid(p, 0, d);
    speedConfig.absoluteEncoder.zeroOffset(chassisAngularOffset);
    this.speedMotor.configure(
        speedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
    this.angleMotor.configure(
        MAXSwerveModule.AngleConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.speedFeedforward = new SimpleMotorFeedforward(ks, kv, ka);

    this.speedEncoder = speedMotor.getEncoder();
    this.angleEncoder = angleMotor.getAbsoluteEncoder();

    this.speedController = speedMotor.getClosedLoopController();
    this.angleController = angleMotor.getClosedLoopController();

    this.speedEncoder.setPosition(0);

  }

  public SwerveModulePosition getPosition() {
    double position = speedEncoder.getPosition();
    Rotation2d angle = new Rotation2d(Rotations.of(angleEncoder.getPosition()));

    if (checkError()) {
      return null;
    }
    return new SwerveModulePosition(position, angle);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        speedEncoder.getVelocity(),
        new Rotation2d(Rotations.of(angleEncoder.getPosition())));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(new Rotation2d(this.angleEncoder.getPosition()));

    setWheelSpeed(desiredState.speedMetersPerSecond);
    setWheelAngle(desiredState.angle.getRotations());
  }

  /**
   * Returns true if errors are detected, false otherwise
   */
  public boolean checkError() {
    boolean retVal = false;
    if (speedMotor.getLastError() != REVLibError.kOk) {
      System.out.println("Error with speed motor: " + speedMotor.getDeviceId());
      System.out.println("Error: " + speedMotor.getLastError());
      speedMotor.clearFaults();
      retVal = true;
    }
    if (angleMotor.getLastError() != REVLibError.kOk) {
      System.out.println("Error with angle motor: " + angleMotor.getDeviceId());
      System.out.println("Error: " + angleMotor.getLastError());
      angleMotor.clearFaults();
      return true;
    }
    return retVal;
  }

  public void setWheelSpeed(double speed) {
    speedController.setReference(
        speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, speedFeedforward.calculate(speed));
  }

  public void setWheelAngle(double angle) {
    angleController.setReference(angle, ControlType.kPosition);
  }

}
