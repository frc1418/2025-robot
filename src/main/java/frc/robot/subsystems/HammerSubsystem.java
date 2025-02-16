// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;;

public class HammerSubsystem extends SubsystemBase {

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/hammer");

  private final NetworkTableEntry ntHolderPiston = table.getEntry("holderExtended");

  private Solenoid holder = new Solenoid(PneumaticsConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, 4);

  private Boolean holderOut = true;

  public HammerSubsystem() {
    holder.set(false);
  }

  public void toggleHolder(){
    holder.toggle();
    holderOut = !holderOut;
  }

  public Command dropArm() {
    return runOnce(
      () -> {
        toggleHolder();
      });
  }

  @Override
  public void periodic() {
    ntHolderPiston.setBoolean(holderOut);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
