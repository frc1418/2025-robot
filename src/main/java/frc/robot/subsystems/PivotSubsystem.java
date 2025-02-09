// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

  private final SparkMax pivotMotor = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  public final SparkMaxConfig motorConfig = new SparkMaxConfig();

  public PivotSubsystem() {
    motorConfig.idleMode(IdleMode.kBrake);
    pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPivot(double speed) {
    pivotMotor.set(speed);
  }

  public Command holdPivot() {
    return new RunCommand(
      () -> {
        setPivot(0.1);
      }, this);
  }

  public Command pivotUp() {
    return new RunCommand(
      () -> {
        setPivot(0.3);
      }, this);
  }

  public Command pivotDown() {
    return new RunCommand(
      () -> {
        setPivot(-0.1);
      }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
