// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignByFieldPose;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AlignByAprilTagLL;
import frc.robot.commands.AlignRot;
import frc.robot.subsystems.ClimbSubsystem;  
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(intakeSubsystem);
  private final PivotSubsystem pivotSubsytem = new PivotSubsystem(intakeSubsystem);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(climbSubsystem);

  CommandJoystick leftJoystick = new CommandJoystick(0);
  CommandJoystick rightJoystick = new CommandJoystick(1);
  CommandJoystick altJoystick = new CommandJoystick(2);

  private final AlignByFieldPose alignByCoralStation = new AlignByFieldPose(driveSubsystem, 16.177, 6.273, 56.7, 0.5, 0, 0.1, 3);
  private final AlignByFieldPose alignByAutoStart = new AlignByFieldPose(driveSubsystem, 17, 5, 0, 0.5, 0, 0.1, 3);
  private final AlignByAprilTagLL alignByAprilTagLL = new AlignByAprilTagLL(driveSubsystem, 0.0, -1.75, 0.3, 0.1, 0.1, 0);
  private final AlignRot alignRot = new AlignRot(this, driveSubsystem, leftJoystick, 0);

  private boolean manualMode = false;
  
  public RobotContainer() {
      NamedCommands.registerCommand("elevatorIntake", elevatorSubsystem.moveElevatorToHeight(0.222));
      NamedCommands.registerCommand("elevatorDeliver", elevatorSubsystem.moveElevatorToHeight(1.01));
      NamedCommands.registerCommand("pivotIntake", pivotSubsytem.setPivot(36));
      NamedCommands.registerCommand("pivotDeliver", pivotSubsytem.setPivot(0));
      NamedCommands.registerCommand("intakeIn", intakeSubsystem.intakeIn());
      NamedCommands.registerCommand("intakeOut", intakeSubsystem.intakeOut());

      configureBindings();

      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser("Test");
      autoChooser.setDefaultOption("Default Path", null);
      SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Positive x moves bot forwards and positive y moves bot to the left
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      driveSubsystem.drive(
        -applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND),
        -applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND),
        -applyDeadband(rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND));
    }, driveSubsystem));

    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.holdElevator());
    pivotSubsytem.setDefaultCommand(pivotSubsytem.holdPivot());
    intakeSubsystem.setDefaultCommand(intakeSubsystem.holdIntake());

    leftJoystick.button(1).onTrue(driveSubsystem.setTempSlowMode(true));
    leftJoystick.button(1).onFalse(driveSubsystem.setTempSlowMode(false));
    leftJoystick.button(2).whileTrue(alignByCoralStation);
    leftJoystick.button(3).whileTrue(alignByAprilTagLL);
    leftJoystick.button(4).whileTrue(alignByAutoStart);
    leftJoystick.button(5).onTrue(driveSubsystem.toggleFieldCentric());
    leftJoystick.button(6).onTrue(driveSubsystem.toggleFastMode());
    leftJoystick.button(7).onTrue(driveSubsystem.toggleLimitDrive());

    rightJoystick.button(1).whileTrue(alignRot);
    rightJoystick.button(2).onTrue(driveSubsystem.resetFieldCentric());
    rightJoystick.button(3).whileTrue(driveSubsystem.getRotError());
    rightJoystick.button(3).onFalse(driveSubsystem.correctError());
    rightJoystick.button(4).whileTrue(driveSubsystem.turtle());

    altJoystick.button(1).whileTrue(elevatorSubsystem.runElevator(-0.2));
    altJoystick.button(2).whileTrue(pivotSubsytem.setPivot(-31));
    altJoystick.button(3).whileTrue(elevatorSubsystem.runElevator(0.2));
    altJoystick.button(4).whileTrue(pivotSubsytem.setPivot(36));
    altJoystick.button(5).whileTrue(checkManualMode(
      pivotSubsytem.pivot(0.075), 
      Commands.parallel(
        elevatorSubsystem.moveElevatorToHeight(0.222),
        pivotSubsytem.setPivot(36))));
    altJoystick.button(6).whileTrue(checkManualMode(
      pivotSubsytem.pivot(-0.075), 
      Commands.parallel(
        elevatorSubsystem.moveElevatorToHeight(1.03),
        pivotSubsytem.setPivot(-31))));    
    altJoystick.button(8).onTrue(toggleManualMode());
    altJoystick.button(9).onTrue(climbSubsystem.toggleAttach());
    altJoystick.button(10).onTrue(climbSubsystem.toggleClimb());
    altJoystick.pov(0).whileTrue(elevatorSubsystem.moveElevatorToHeight(1.01));
    altJoystick.pov(90).whileTrue(intakeSubsystem.intakeIn());
    altJoystick.pov(180).whileTrue(elevatorSubsystem.moveElevatorToHeight(0.222));
    altJoystick.pov(270).whileTrue(intakeSubsystem.intakeOut());
  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
  }

  public void resetLockRot() {
    driveSubsystem.resetLockRot();
  }

  public Command toggleManualMode() {
    return Commands.runOnce(() -> {
      manualMode = !manualMode;
      System.out.println("Manual mode: " + manualMode);
    });
  }

  public Command checkManualMode(Command manualCommand, Command autoCommand) {
    return new RunCommand(() -> {
      if (manualMode) {
        manualCommand.schedule();
      }
      else {
        autoCommand.schedule();
      }
    }).finallyDo(interupted -> {
      manualCommand.cancel();
      autoCommand.cancel();
    });
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
