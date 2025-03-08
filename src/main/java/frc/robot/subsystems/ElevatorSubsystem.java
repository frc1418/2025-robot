// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/elevator");

  private final NetworkTableEntry ntElevatorHeight = table.getEntry("elevatorHeight");
  private final NetworkTableEntry ntElevatorSpeed = table.getEntry("elevatorSpeed");
  private final NetworkTableEntry ntElevatorAtBottom = table.getEntry("elevatorAtBottom");

  private final SparkMax motor1 = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final AbsoluteEncoder elevatorEncoder;
  private IntakeSubsystem intakeSubsystem;

  private DigitalInput zeroSwitch = new DigitalInput(9);
  private double zeroCounter= 0;

  private final PIDController elevatorController = new PIDController(ElevatorConstants.kP, 0, ElevatorConstants.kD);
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(0.85);

  private double encoderScalar = ElevatorConstants.ENCODER_SCALAR;
  private double lastHeight;
  private double initialHeight;
  private double heightValue;
  private double kG = ElevatorConstants.kG;

  public ElevatorSubsystem(IntakeSubsystem intakeSubsystem) {
    motorConfig.idleMode(IdleMode.kBrake);
    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorEncoder = motor1.getAbsoluteEncoder();
    lastHeight = elevatorEncoder.getPosition();
    initialHeight = (elevatorEncoder.getPosition()+ElevatorConstants.ELEVATOR_OFFSET)/encoderScalar;
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command holdElevator() {
    return new RunCommand(
      () -> {
        if (heightValue > 0.03) {
          moveElevator(kG);
        }
        else {
          moveElevator(0);
        }
      }, this);
  }

  public Command runElevator(double speed) {
    return new RunCommand(
      () -> {
        moveElevator(speed+kG);
      }, this);
  }

  public Command moveElevatorToHeight(double height) {
    return new RunCommand(
      () -> {
        setElevatorLocation(height);
      }, this);
  }

  public double getHeight() {
    return heightValue;
  }

  public Boolean isSafeL3() {
    return heightValue < 0.6;
  }

  public Boolean isTop() {
    return heightValue > 1.02;
  }

  public Boolean isMiddle() {
    return heightValue < 0.57;
  }

  public Boolean isKindaLow() {
    return heightValue < 0.24;
  }

  public Boolean isLow() {
    return heightValue < 0.05;
  }

  public void moveElevator(double speed) {
    speed = speedLimiter.calculate(speed);
    motor1.set(speed);
    motor2.set(speed);
  }

  public void setElevatorLocation(double height) {
    double error = height - heightValue;
    double speed;
    if (Math.abs(error) < 0.0025) {
      speed = kG;
    }
    else {
      speed = elevatorController.calculate(heightValue, height)+ElevatorConstants.kV*Math.signum(error)+kG;
    }
    if (Math.abs(speed) > ElevatorConstants.maxSpeed) {
      speed = Math.signum(speed)*ElevatorConstants.maxSpeed;
    }
    moveElevator(speed);
  }

  public void updateElevator() {
    double rawPosition = elevatorEncoder.getPosition();
    double deltaHeight = rawPosition - lastHeight;
    while (deltaHeight < -0.5) {
      deltaHeight += 1;
    }
    while (deltaHeight > 0.5) {
      deltaHeight -= 1;
    }
    lastHeight += deltaHeight;
    double normalizedHeight = lastHeight/encoderScalar;
    heightValue = normalizedHeight - initialHeight;
    ntElevatorHeight.setDouble(heightValue);
    ntElevatorSpeed.setDouble(elevatorEncoder.getVelocity());
  }

  public void updateFF() {
    if (intakeSubsystem.getHasCoral()) {
      kG = ElevatorConstants.kG+ElevatorConstants.kCoral;
    }
    else {
      kG = ElevatorConstants.kG;
    }
  }

  public Command reZero() {
    return Commands.runOnce(() -> {
      zero();
    });
  }

  public void zero() {
    initialHeight = (elevatorEncoder.getPosition()+ElevatorConstants.ELEVATOR_OFFSET)/encoderScalar;
  }

  public Command smoothControl(double smoothValue) {
    return new RunCommand(() -> {
      speedLimiter.reset(smoothValue);
    });
  }

  @Override
  public void periodic() {
    updateElevator();
    updateFF();
    ntElevatorAtBottom.setBoolean(zeroSwitch.get());
    if (zeroSwitch.get()) {
      zeroCounter++;
      if (zeroCounter > 10) {
        zero();
        zeroCounter = 0;
      }
    }
    else {
      zeroCounter = 0;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
