// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Intake extends SequentialCommandGroup {
  public Intake(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(
        pivotSubsystem.setPivot(36),
        elevatorSubsystem.moveElevatorToHeight(0.222));
        intakeSubsystem.setIntake(0.25);
  }
}
