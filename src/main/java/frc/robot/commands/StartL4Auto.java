// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StartL4Auto extends SequentialCommandGroup {
  public StartL4Auto(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem, RobotBase robot) {
    addRequirements(pivotSubsystem, elevatorSubsystem, intakeSubsystem);
    addCommands(
      ledSubsystem.dontMoveColor(),
      Commands.deadline(
        Commands.waitSeconds(0.01),
        elevatorSubsystem.smoothControl(-0.01)),
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isTop), 
        pivotSubsystem.setPivot(60),
        elevatorSubsystem.moveElevatorToHeight(1.1))
    );
  }
}
