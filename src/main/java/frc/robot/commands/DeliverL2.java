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

public class DeliverL2 extends SequentialCommandGroup {
  public DeliverL2(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem, RobotBase robot) {
    addCommands(
      ledSubsystem.dontMoveColor(),
      Commands.deadline(
        Commands.waitSeconds(1), 
        pivotSubsystem.setPivot(65),
        elevatorSubsystem.moveElevatorToHeight(0.42)),
      Commands.deadline(
        Commands.waitSeconds(1),
        pivotSubsystem.setPivot(-50),
        elevatorSubsystem.moveElevatorToHeight(0.42)),
      ledSubsystem.allianceColor(),
      Commands.deadline(
      Commands.waitUntil(robot::isAutonomous),
      pivotSubsystem.setPivot(-50),
      elevatorSubsystem.moveElevatorToHeight(0.42)) 
    );
  }
}
