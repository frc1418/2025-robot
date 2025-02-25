// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(PivotConstants.TALON_MOTOR_ID);

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/pivot");
  private final NetworkTableEntry ntPivotAmount = table.getEntry("pivotAmountDegrees");

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(6);

  private double lastPivot;
  private double initialPivot = PivotConstants.PIVOT_OFFSET;
  private double pivotValue;
  private double encoderScalar = PivotConstants.ENCODER_SCALAR;
  private double kG = PivotConstants.kG;

  private IntakeSubsystem intakeSubsystem;

  private final PIDController pivotController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);

  public PivotSubsystem(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    lastPivot = pivotEncoder.get()-1;

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
    double force = maintainAngle(posDegrees);

    double error = posDegrees - pivotValue*360;
    if (Math.abs(error) < PivotConstants.pivotTolerance) {
      pivotMotor.set(force);
      resetPivotPID();
    }
    else {
      System.out.println(pivotController.calculate(pivotValue*360, posDegrees));
      force -= pivotController.calculate(pivotValue*360, posDegrees);
      force -= Math.signum(error) * PivotConstants.kV;
      System.out.println("force: " + force);
      System.out.println("PID: " + force);
      if (Math.abs(force) > 0.1) {
        force = Math.signum(force)*0.1;
      }
      pivotMotor.set(force);
    }
  }

  public void resetPivot() {
    pivotController.reset();
  }

  public void updatePivot() {
    double rawPosition = pivotEncoder.get();
    double deltaPivot = rawPosition - lastPivot;
    while (deltaPivot < -0.5) {
      deltaPivot += 1;
    }
    while (deltaPivot > 0.5) {
      deltaPivot -= 1;
    }
    lastPivot += deltaPivot;
    double normalizedPivot = lastPivot/encoderScalar;
    pivotValue = normalizedPivot - initialPivot;
    ntPivotAmount.setDouble(pivotValue*360);
  }

  public void updateFF() {
    if (intakeSubsystem.getHasCoral()) {
      kG = PivotConstants.kG+PivotConstants.kCoral;
    }
    else {
      kG = PivotConstants.kG;
    }
  }

  public double getPivotDegrees() {
    return pivotValue*360;
  }

  public double maintainAngle(double pivotDegrees) {
    double force = -(PivotConstants.kConstant+kG*Math.cos(pivotDegrees*2*Math.PI));
    return force;
  }

  public Command holdPivot() {
    return new RunCommand(
      () -> {
        pivotMotor.set(maintainAngle(pivotValue));
      }, this);
  }

  public Command pivot(double speed) {
    return new RunCommand(
      () -> {
        pivotMotor.set(speed+maintainAngle(pivotValue));
      }, this);
  }

  public Command setPivot(double pivotDegrees) {
    return new RunCommand(
      () -> {
        setPivotLocation(pivotDegrees);
      }, this);
  }

  public Command resetPivotPID() {
    return Commands.runOnce(
      () -> {
        pivotController.reset();
        System.out.println("reset");
      }, this);
  }

  @Override
  public void periodic() {
    updatePivot();
    updateFF();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
