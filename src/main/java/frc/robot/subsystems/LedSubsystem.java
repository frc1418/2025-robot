// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedColor;

public class LedSubsystem extends SubsystemBase {

    public LedColor color;
    private Boolean hasPriority = false;
    private Spark blinkin = new Spark(0);

    public LedSubsystem() {        
      setAllianceColor();
    }

    @Override
    public void periodic() {
        try {
          blinkin.set(color.color());
        } catch (Exception e) {
          System.out.println("Error: " + e.getStackTrace());
        }
    }

    public void setAllianceColor() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue)
        color = LedColor.BLUE_ALLIANCE;
      else
        color = LedColor.RED_ALLIANCE;
      }
      hasPriority = false;
    }

    public void setAligningColor() {
        color = LedColor.ALIGNING;
        hasPriority = true;
    }

    public void setAlignedColor() {
        color = LedColor.ALIGNED;
        hasPriority = true;
    }

    public void setDontMoveColor() {
        color = LedColor.DONT_MOVE;
        hasPriority = true;
    }

    public void setCoralInColor() {
      if (!hasPriority) {
        color = LedColor.CORAL_IN;
      }
    }

    public void resetColor() {
        if (!hasPriority) {
            setAllianceColor();
        }
    }

    public Command dontMoveColor() {
      return Commands.runOnce(
        () -> {
          setDontMoveColor();
        });
    }

    public Command allianceColor() {
      return Commands.runOnce(
        () -> {
          setAllianceColor();
        });
    }
}