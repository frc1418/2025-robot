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

public class DropFromL4 extends SequentialCommandGroup {
  public DropFromL4(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem, RobotBase robot) {
    addRequirements(pivotSubsystem, elevatorSubsystem, intakeSubsystem);
    addCommands(
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isKindaLowAuto), 
        pivotSubsystem.setPivot(95),
        elevatorSubsystem.moveElevatorToHeight(0.23)),
      ledSubsystem.allianceColor(),
      Commands.deadline(
      Commands.waitUntil(robot::isAutonomous),
      pivotSubsystem.setPivot(95),
      elevatorSubsystem.moveElevatorToHeight(0.23))
    );
  }
}
