// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/elevator");

  private final NetworkTableEntry ntElevatorHeight = table.getEntry("elevatorHeight");
  private final NetworkTableEntry ntElevatorSpeed = table.getEntry("elevatorSpeed");
  private final NetworkTableEntry ntElevatorAtBottom = table.getEntry("elevatorAtBottom");

  private final SparkMax motor1 = new SparkMax(
      ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
  private final SparkClosedLoopController motor1Controller = motor1.getClosedLoopController();
  private final SparkMax motor2 = new SparkMax(
      ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final RelativeEncoder encoder = motor1.getEncoder();

  private DigitalInput zeroSwitch = new DigitalInput(9);

  // TODO get kS and kA values
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      0.0, ElevatorConstants.kG, ElevatorConstants.kV, 0.0);
  private final TrapezoidProfile profile = new TrapezoidProfile(
      new Constraints(ElevatorConstants.maxSpeed, ElevatorConstants.maxAccel));

  private double heightValue;

  public ElevatorSubsystem() {
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.closedLoop.pid(ElevatorConstants.kP, 0.0, ElevatorConstants.kD);
    motorConfig.encoder.positionConversionFactor(1.0 / 6.0);
    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig.follow(motor1);
    motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    new Trigger(zeroSwitch::get).onTrue(
        Commands.runOnce(() -> encoder.setPosition(0.0)).ignoringDisable(true));
  }

  private TrapezoidProfile.State setpoint = new State();

  public Command moveElevatorToHeight(double height) {
    return run(() -> {
      setpoint = profile.calculate(0.20, setpoint, new State(height, 0));
      motor1Controller.setReference(
          setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
          feedforward.calculate(setpoint.velocity));
    });
  }

  public double getHeight() {
    return encoder.getPosition() / 4.52;
  }

  public boolean isSafeL3() {
    return getHeight() < 0.6;
  }

  public boolean isTop() {
    return getHeight() > 0.99;
  }

  public boolean isMiddle() {
    return getHeight() < 0.57;
  }

  public boolean isKindaLow() {
    //TODO: Raise value to speed up elevator dropping
    return getHeight() < 0.26;
  }

  public boolean isKindaLowAuto() {
    return getHeight() < 0.33;
  }

  public boolean isLow() {
    return getHeight() < 0.05;
  }

  @Override
  public void periodic() {
    ntElevatorHeight.setDouble(getHeight());
    ntElevatorSpeed.setDouble(encoder.getVelocity());
    ntElevatorAtBottom.setBoolean(zeroSwitch.get());
  }

}
