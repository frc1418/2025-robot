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

public class L4Auto extends SequentialCommandGroup {
  public L4Auto(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem, RobotBase robot) {
    addRequirements(pivotSubsystem, elevatorSubsystem, intakeSubsystem);
    addCommands(
      ledSubsystem.dontMoveColor(),
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isTop),
        pivotSubsystem.setPivot(60),
        elevatorSubsystem.moveElevatorToHeight(1.1)),
      Commands.deadline(
        Commands.waitSeconds(0.7),
        elevatorSubsystem.moveElevatorToHeight(1.1),
        pivotSubsystem.setPivot(-50)),
      Commands.deadline(
        Commands.waitSeconds(0.4), 
        intakeSubsystem.intakeOut(),
        pivotSubsystem.setPivot(-50),
        elevatorSubsystem.smoothControl(0.075)),
      Commands.deadline(
        Commands.waitUntil(elevatorSubsystem::isMiddle), 
        intakeSubsystem.holdIntake(), 
        pivotSubsystem.setPivot(70),
        elevatorSubsystem.moveElevatorToHeight(0.23))
    );
  }
}
