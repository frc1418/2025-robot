// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.PneumaticsConstants;

;

public class ClimbSubsystem extends SubsystemBase {

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/climb");

  private final NetworkTableEntry ntClimbPistons = table.getEntry("footExtended");
  private final NetworkTableEntry ntAttachPistons = table.getEntry("handsExtended");
  private final NetworkTableEntry ntPressure = table.getEntry("pressure");


  PneumaticHub ph = new PneumaticHub(PneumaticsConstants.PNEUMATICS_HUB_ID);


  private final DoubleSolenoid climbSolenoid = new DoubleSolenoid(
      PneumaticsConstants.PNEUMATICS_HUB_ID,
      PneumaticsModuleType.REVPH,
      PneumaticsConstants.CLIMB_SOLENOID_FORWARD,
      PneumaticsConstants.CLIMB_SOLENOID_REVERSE);

  private final DoubleSolenoid attachSolenoid = new DoubleSolenoid(
      PneumaticsConstants.PNEUMATICS_HUB_ID,
      PneumaticsModuleType.REVPH,
      PneumaticsConstants.ATTACH_SOLENOID_FORWARD,
      PneumaticsConstants.ATTACH_SOLENOID_REVERSE);

  public ClimbSubsystem() {
    climbSolenoid.set(DoubleSolenoid.Value.kReverse);
    attachSolenoid.set(DoubleSolenoid.Value.kReverse);

    // Disable compressor until the end of the match
    ph.disableCompressor();
    RobotModeTriggers.teleop().and(() -> DriverStation.getMatchTime() > 90).or(
        RobotModeTriggers.test()).onTrue(
        Commands.runOnce(() -> ph.enableCompressorAnalog(100, 120))
    );

  }

  public boolean isClimbing() {
    return climbSolenoid.get() == Value.kForward;
  }

  public void climbToggle() {
    if (attachSolenoid.get() == Value.kForward) {
      climbSolenoid.toggle();
    }
  }

  public void climbExtend() {
    climbSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void climbRetract() {
    climbSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void attachToggle() {
    attachSolenoid.toggle();
  }

  public Command toggleClimb() {
    return runOnce(this::climbToggle);
  }

  public Command toggleAttach() {
    return runOnce(this::attachToggle);
  }


  @Override
  public void periodic() {
    ntClimbPistons.setBoolean(climbSolenoid.get() == Value.kForward);
    ntAttachPistons.setBoolean(attachSolenoid.get() == Value.kForward);
    ntPressure.setDouble(ph.getPressure(0));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once pßer scheduler run during simulation
  }
}
