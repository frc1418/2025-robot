// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();

  private DigitalInput coralSwitch = new DigitalInput(8);

  private boolean hasCoral = false;


  public IntakeSubsystem() {
    motorConfig.idleMode(IdleMode.kBrake);
    intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command holdIntake() {
    return new RunCommand(
      () -> {
        setIntake(0.05);
      }, this);
  }

  public Command intakeOut() {
    return new RunCommand(
      () -> {
        setIntake(-0.25);
      }, this);
  }

  public Command intakeIn() {
    return new RunCommand(
      () -> {
        setIntake(0.25);
      }, this);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public Boolean getHasCoral() {
    return hasCoral;
  }


  @Override
  public void periodic() {
    if (coralSwitch.get()) {
      hasCoral = true;
      System.out.println("true");
    }
    else {
      hasCoral = false;
      System.out.println("false");
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
