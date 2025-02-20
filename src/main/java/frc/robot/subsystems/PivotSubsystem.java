// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController pivotController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);

  public PivotSubsystem() {
    pivotMotor.setPosition(PivotConstants.pivotOffsetFromStart);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(talonConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void setPivotLocation(double posDegrees) {
    double force = -pivotController.calculate(pivotMotor.getPosition().getValueAsDouble()*360, posDegrees);
    force = -(PivotConstants.kLeverage*Math.cos(pivotMotor.getPosition().getValueAsDouble()*Math.PI*2)+PivotConstants.kG);
    pivotMotor.set(force);
  }

  public void resetPivot() {
    pivotController.reset();
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
        pivotMotor.set(-0.03);
      }, this);
  }

  public Command pivotUp() {
    return new RunCommand(
      () -> {
        pivotMotor.set(-0.1);
      }, this);
  }

  public Command pivotDown() {
    return new RunCommand(
      () -> {
        pivotMotor.set(0.02);
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
