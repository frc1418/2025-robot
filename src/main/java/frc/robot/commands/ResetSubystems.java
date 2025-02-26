// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ResetSubystems extends SequentialCommandGroup {
  public ResetSubystems(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem) {
    addCommands(
        Commands.deadline(
            Commands.waitSeconds(3), 
            pivotSubsystem.setPivot(65),
            elevatorSubsystem.moveElevatorToHeight(0.2)),
        Commands.parallel(
            pivotSubsystem.setPivot(95),
            elevatorSubsystem.moveElevatorToHeight(0))
    );
  }
}
