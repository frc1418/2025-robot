// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Reset extends SequentialCommandGroup {
  public Reset(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, LedSubsystem ledSubsystem, RobotBase robot) {
    addCommands(
      ledSubsystem.dontMoveColor(),
      Commands.deadline(
        Commands.waitSeconds(0.01),
        elevatorSubsystem.smoothControl(0.125)),
      Commands.deadline(
          Commands.waitUntil(pivotSubsystem::isSafe), 
          pivotSubsystem.setPivot(69),
          elevatorSubsystem.smoothControl(0.125)),
      Commands.deadline(
          Commands.waitUntil(elevatorSubsystem::isMiddle), 
          pivotSubsystem.setPivot(69),
          elevatorSubsystem.moveElevatorToHeight(0.2)),
      Commands.deadline(
          Commands.waitUntil(elevatorSubsystem::isKindaLow), 
          pivotSubsystem.setPivot(95),
          elevatorSubsystem.moveElevatorToHeight(0.2)),
      Commands.deadline(
          Commands.waitUntil(elevatorSubsystem::isLow),
          pivotSubsystem.setPivot(95),
          elevatorSubsystem.moveElevatorToHeight(-0.1)),
      ledSubsystem.allianceColor(),
      Commands.waitUntil(robot::isAutonomous)
    );
  }
}
