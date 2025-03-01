// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedColor;

public class LedSubsystem extends SubsystemBase {

    public LedColor color;
    private Spark blinkin = new Spark(8);

    public LedSubsystem() {        
      setAllianceColor();
    }

    @Override
    public void periodic() {
        blinkin.set(color.color());
    }

    public void setAllianceColor() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue)
        color = LedColor.BLUE_ALLIANCE;
      else
        color = LedColor.RED_ALLIANCE;
      }
    }

    public void setBalancingColor() {
        color = LedColor.BALANCING;
    }

    public void setDockedColor() {
        color = LedColor.DOCKED;
    }

    public void setGrabberOpen() {
        color = LedColor.GRABBER_OPEN;
    }

    public void setGrabberClosed() {
        color = LedColor.GRABBER_CLOSED;
    }
}