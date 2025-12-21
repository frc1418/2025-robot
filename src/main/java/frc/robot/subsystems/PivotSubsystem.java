// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(PivotConstants.TALON_MOTOR_ID);

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/pivot");
  private final NetworkTableEntry ntPivotAmount = table.getEntry("pivotAmountDegrees");

  private final PositionVoltage pivotControl = new PositionVoltage(0.0);

  public PivotSubsystem() {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Slot0.kP = PivotConstants.kP;
    talonConfig.Slot0.kI = PivotConstants.kI;
    talonConfig.Slot0.kD = PivotConstants.kD;
    talonConfig.Slot0.kG = PivotConstants.kG;

    // TODO find initial position. 0 is horizontal
    pivotMotor.setPosition(Degrees.of(-10));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(talonConfig);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }


  public double getPivotDegrees() {
    return pivotMotor.getPosition().getValueAsDouble() * 360;
  }

  public boolean isSafe() {
    return (getPivotDegrees() < 72 && getPivotDegrees() > 56);
  }

  public Command setPivot(double pivotDegrees) {
    return setPivot(Degrees.of(pivotDegrees));
  }

  public Command setPivot(Angle pivotPosition) {
    return run(() -> pivotMotor.setControl(pivotControl.withPosition(pivotPosition)));
  }

  @Override
  public void periodic() {
    ntPivotAmount.setDouble(pivotMotor.getPosition().getValueAsDouble() * 360);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
