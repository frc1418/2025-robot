// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(PivotConstants.TALON_MOTOR_ID);

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/pivot");
  private final NetworkTableEntry ntPivotAmount = table.getEntry("pivotAmountDegrees");
  private final NetworkTableEntry ntPivotSpeed = table.getEntry("pivotSpeedDegreesPerSecond");
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public PivotSubsystem() {
    pivotMotor.setPosition(PivotConstants.pivotOffsetFromStart);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FeedbackConfigs feedbackConfigs = talonConfig.Feedback;
    feedbackConfigs.SensorToMechanismRatio = PivotConstants.pivotRotationsToArmRotations;
    MotionMagicConfigs motionMagicConfigs = talonConfig.MotionMagic;
    motionMagicConfigs
      .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.1)) // 1 (mechanism) rotations per 10 second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0.1)) // Take approximately 0.5 seconds to reach max vel
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(20)); // Take approximately 0.1 seconds to reach max accel 

    // TODO: Tune these values they are prob way too high
    Slot0Configs slot0 = talonConfig.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(talonConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void setPivot(double speed) {
    pivotMotor.set(speed);
  }

  public void setPivotLocation(double posRadians) {
    double posRotations = posRadians / (2*Math.PI);
    pivotMotor.setControl(motionMagicVoltage.withPosition(posRotations).withSlot(0));
  }

  public void updatePivot() {
    ntPivotAmount.setDouble(pivotMotor.getPosition().getValueAsDouble()*360);
    ntPivotSpeed.setDouble(pivotMotor.getVelocity().getValueAsDouble()*360);
  }

  public double getPivotAmount() {
    return ntPivotAmount.getDouble(getPivotAmount());
  }

  public void resetPivotEncoder() {
    pivotMotor.setPosition(0);
    ntPivotAmount.setDouble(0);
}

  public Command holdPivot() {
    return new RunCommand(
      () -> {
        setPivot(-0.03);
      }, this);
  }

  public Command pivotUp() {
    return new RunCommand(
      () -> {
        setPivot(-0.1);
      }, this);
  }

  public Command pivotDown() {
    return new RunCommand(
      () -> {
        setPivot(0.02);
      }, this);
  }

  @Override
  public void periodic() {
    updatePivot();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
