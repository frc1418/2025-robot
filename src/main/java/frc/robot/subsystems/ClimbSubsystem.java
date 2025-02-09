// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbSubsystem extends SubsystemBase {

  private DoubleSolenoid leftSolenoid = new DoubleSolenoid(
    ClimberConstants.PNEUMATICS_HUB_ID, 
    PneumaticsModuleType.REVPH, 
    ClimberConstants.LEFT_SOLENOID_FORWARD, 
    ClimberConstants.LEFT_SOLENOID_REVERSE);
  private DoubleSolenoid rightSolenoid = new DoubleSolenoid(
    ClimberConstants.PNEUMATICS_HUB_ID, 
    PneumaticsModuleType.REVPH, 
    ClimberConstants.RIGHT_SOLENOID_FORWARD, 
    ClimberConstants.RIGHT_SOLENOID_REVERSE);
  private Boolean pistonsOut = false;

  public ClimbSubsystem() {}

  public void toggle(){
    leftSolenoid.toggle();
    rightSolenoid.toggle();
    System.out.println(leftSolenoid.get());
    pistonsOut = !pistonsOut;
  }

  public void extend(){
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
    pistonsOut = true;
  }

  public void retract(){
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    pistonsOut = false;
  }

  public Command togglePistons() {
    return runOnce(
      () -> {
        toggle();
      });
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
