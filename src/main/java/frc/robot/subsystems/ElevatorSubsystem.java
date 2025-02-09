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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax motor1 = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
  public final SparkMaxConfig motorConfig = new SparkMaxConfig();


  public ElevatorSubsystem() {
    motorConfig.idleMode(IdleMode.kBrake);
    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command holdElevator() {
    return new RunCommand(
      () -> {
        moveElevator(0.1);
      }, this);
  }

  public Command moveElevatorUp() {
    return new RunCommand(
      () -> {
        moveElevator(0.3);
      }, this);
  }

  public Command moveElevatorDown() {
    return new RunCommand(
      () -> {
        moveElevator(0);
      }, this);
  }

  public void moveElevator(double speed) {
    motor1.set(speed);
    motor2.set(speed);
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
