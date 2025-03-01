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

public class Intake extends SequentialCommandGroup {
  public Intake(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem, RobotBase robot) {
    addCommands(
      ledSubsystem.dontMoveColor(),
      Commands.deadline(
        Commands.waitUntil(intakeSubsystem::getHasCoral), 
        pivotSubsystem.setPivot(36),
        elevatorSubsystem.moveElevatorToHeight(0.222),
        intakeSubsystem.intakeIn()),
      Commands.deadline(
        Commands.waitUntil(pivotSubsystem::isSafe), 
        pivotSubsystem.setPivot(61.5),
        elevatorSubsystem.smoothControl()),
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isMiddle), 
        pivotSubsystem.setPivot(61.5),
        elevatorSubsystem.moveElevatorToHeight(0.2)),
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isKindaLow), 
        pivotSubsystem.setPivot(95),
        elevatorSubsystem.moveElevatorToHeight(0.2)),
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isLow),
        pivotSubsystem.setPivot(95),
        elevatorSubsystem.moveElevatorToHeight(-0.05)),
      ledSubsystem.allianceColor(),
      Commands.waitUntil(robot::isAutonomous)    
    );
  }
}
