// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;;

public class ClimbSubsystem extends SubsystemBase {

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/climb");

  private final NetworkTableEntry ntClimbPistons = table.getEntry("footExtended");
  private final NetworkTableEntry ntAttachPistons = table.getEntry("handsExtended");
  private final NetworkTableEntry ntPressure = table.getEntry("pressure");


  PneumaticHub ph = new PneumaticHub(PneumaticsConstants.PNEUMATICS_HUB_ID);

  private DoubleSolenoid climbSolenoid = new DoubleSolenoid(
    PneumaticsConstants.PNEUMATICS_HUB_ID, 
    PneumaticsModuleType.REVPH, 
    PneumaticsConstants.CLIMB_SOLENOID_FORWARD, 
    PneumaticsConstants.CLIMB_SOLENOID_REVERSE);

  private DoubleSolenoid attachSolenoid = new DoubleSolenoid(
    PneumaticsConstants.PNEUMATICS_HUB_ID, 
    PneumaticsModuleType.REVPH, 
    PneumaticsConstants.ATTACH_SOLENOID_FORWARD,
    PneumaticsConstants.ATTACH_SOLENOID_REVERSE);

  private Boolean climbPistonsOut = false;
  private Boolean attachPistonsOut = true;

  public ClimbSubsystem() {
    climbSolenoid.set(DoubleSolenoid.Value.kReverse);
    attachSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void climbToggle(){
    climbSolenoid.toggle();
    climbPistonsOut = !climbPistonsOut;
  }

  public void climbExtend(){
    climbSolenoid.set(DoubleSolenoid.Value.kForward);
    climbPistonsOut = true;
  }

  public void climbRetract(){
    climbSolenoid.set(DoubleSolenoid.Value.kReverse);
    climbPistonsOut = false;
  }

  public void attachToggle() {
    attachSolenoid.toggle();
    System.out.println(attachSolenoid.get());
    attachPistonsOut = !attachPistonsOut;
  }

  public Command toggleClimb() {
    return runOnce(
      () -> {
        climbToggle();
      });
  }

  public Command toggleAttach() {
    return runOnce(
      () -> {
        attachToggle();
      });
  }


  @Override
  public void periodic() {
    ph.enableCompressorAnalog(100,120);
    ntClimbPistons.setBoolean(climbPistonsOut);
    ntAttachPistons.setBoolean(attachPistonsOut);
    ntPressure.setDouble(ph.getPressure(0));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
