// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/intake");

  private final NetworkTableEntry ntHasCoral = table.getEntry("hasCoral");

  private final SparkFlex intakeMotor = new SparkFlex(
      IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();

  private DigitalInput coralSwitch = new DigitalInput(8);

  private boolean hasCoral = false;


  public IntakeSubsystem(LedSubsystem leds) {
    motorConfig.idleMode(IdleMode.kBrake);
    intakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    new Trigger(coralSwitch::get).onTrue(
        Commands.runOnce(leds::setCoralInColor).ignoringDisable(true)).onFalse(
        Commands.runOnce(leds::resetColor).ignoringDisable(true));
  }

  public Command holdIntake() {
    return run(() -> {
      intakeMotor.getClosedLoopController().setReference(20, ControlType.kCurrent);
    });
  }

  public Command intakeOut() {
    return run(() -> {
      intakeMotor.set(-0.15);
    });
  }

  /* TODO test out current control for holding gamepieces. Because current is directly proprtional to torque,
  this is effectively controlling the hold power. You can even use the same control for intaking and holding
  */
  public Command intakeIn() {
    return run(() -> {
      intakeMotor.getClosedLoopController().setReference(20, ControlType.kCurrent);
    });
  }

  public boolean getHasCoral() {
    return coralSwitch.get();
  }


  @Override
  public void periodic() {
    ntHasCoral.setBoolean(hasCoral);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
