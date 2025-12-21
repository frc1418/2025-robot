// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakeAuto extends SequentialCommandGroup {

  public IntakeAuto(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem) {
    addCommands(
        ledSubsystem.dontMoveColor(),
        pivotSubsystem.setPivot(36)
                      .alongWith(
                          elevatorSubsystem.moveElevatorToHeight(0.24),
                          intakeSubsystem.intakeIn())
                      .until(intakeSubsystem::getHasCoral),
        ledSubsystem.allianceColor()
    );
  }
}
