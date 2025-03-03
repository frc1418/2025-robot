// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignByFieldPose;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.DeliverL1;
import frc.robot.commands.DeliverL2;
import frc.robot.commands.DeliverL3;
import frc.robot.commands.DeliverL4;
import frc.robot.commands.Intake;
import frc.robot.commands.Reset;
import frc.robot.commands.AlignRot;
import frc.robot.subsystems.ClimbSubsystem;  
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import java.util.Set;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;

  private final LedSubsystem ledSubsystem = new LedSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(ledSubsystem);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(intakeSubsystem);
  private final PivotSubsystem pivotSubsytem = new PivotSubsystem(intakeSubsystem);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(climbSubsystem);

  CommandJoystick leftJoystick = new CommandJoystick(0);
  CommandJoystick rightJoystick = new CommandJoystick(1);
  CommandJoystick altJoystick = new CommandJoystick(2);

  // private final AlignByAprilTagLL alignByAprilTagLL = new AlignByAprilTagLL(driveSubsystem, 0.0, -1.75, 0.3, 0.1, 0.1, 0);
  // private final AlignByFieldPose alignByReefB = new AlignByFieldPose(driveSubsystem, 14.443, 4.12, 180, 0.5, 0, 0.1, 3);
  // L4 back offset 0.553
  private final AlignByFieldPose alignByReefLeft = new AlignByFieldPose(driveSubsystem, ledSubsystem, 0.11, 0.55, 0, 0.5, 0, 0.1, 0, 3, true);
  private final AlignByFieldPose alignByReefRight = new AlignByFieldPose(driveSubsystem, ledSubsystem, 0.11, 0.55, 0, 0.5, 0, 0.1, 0, 3, false);
  private final AlignByFieldPose alignByIntake = new AlignByFieldPose(driveSubsystem, ledSubsystem, 0, 0.665, 0, 0.5, 0, 0.1, 0, 3, false);
  // private final AlignByFieldPose alignByRightIntake = new AlignByFieldPose(driveSubsystem, 15.328, 4.45, 53.33, 0.5, 0, 0.1, 3);
  private final AlignRot alignRotBackward = new AlignRot(this, driveSubsystem, leftJoystick, 180);
  private final AlignRot alignRotForward = new AlignRot(this, driveSubsystem, leftJoystick, 0);

  private final DeliverL4 deliverL4;
  private final DeliverL3 deliverL3;
  private final DeliverL2 deliverL2;
  private final DeliverL1 deliverL1;
  private final Intake intake;
  private final Reset reset;

  private Boolean manualMode = false;
  
  public RobotContainer(RobotBase robot) {
    deliverL4 = new DeliverL4(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL3 = new DeliverL3(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL2 = new DeliverL2(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL1 = new DeliverL1(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    intake = new Intake(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    reset = new Reset(pivotSubsytem, elevatorSubsystem, ledSubsystem, robot);

    NamedCommands.registerCommand("elevatorIntake", elevatorSubsystem.moveElevatorToHeight(0.222));
    NamedCommands.registerCommand("elevatorDeliver", elevatorSubsystem.moveElevatorToHeight(1.01));
    NamedCommands.registerCommand("pivotIntake", pivotSubsytem.setPivot(36));
    NamedCommands.registerCommand("pivotDeliver", pivotSubsytem.setPivot(-31));
    NamedCommands.registerCommand("intakeIn", intakeSubsystem.intakeIn());
    NamedCommands.registerCommand("intakeOut", intakeSubsystem.intakeOut());
    NamedCommands.registerCommand("deliverL1", deliverL1);
    NamedCommands.registerCommand("deliverL2", deliverL2);
    NamedCommands.registerCommand("deliverL3", deliverL3);
    NamedCommands.registerCommand("deliverL4", deliverL4);
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("reset", reset);

    new EventTrigger("elevatorIntake").whileTrue(elevatorSubsystem.moveElevatorToHeight(0.222));
    new EventTrigger("elevatorDeliver").whileTrue(elevatorSubsystem.moveElevatorToHeight(1.03));
    new EventTrigger("pivotIntake").whileTrue(pivotSubsytem.setPivot(36));
    new EventTrigger("pivotDeliver").whileTrue(pivotSubsytem.setPivot(-31));
    new EventTrigger("intakeIn").whileTrue(intakeSubsystem.intakeIn());
    new EventTrigger("intakeOut").whileTrue(intakeSubsystem.intakeOut());
    new EventTrigger("deliverL1").whileTrue(deliverL1);
    new EventTrigger("deliverL2").whileTrue(deliverL2);
    new EventTrigger("deliverL3").whileTrue(deliverL3);
    new EventTrigger("deliverL4").whileTrue(deliverL4);
    new EventTrigger("intake").whileTrue(intake);
    new EventTrigger("reset").whileTrue(reset);

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

    leftJoystick.button(1).whileTrue(checkAprilTagNumber(alignByReefLeft, alignByIntake));
    leftJoystick.button(2).onTrue(driveSubsystem.setTempSlowMode(true));
    leftJoystick.button(2).onFalse(driveSubsystem.setTempSlowMode(false));
    leftJoystick.button(5).onTrue(driveSubsystem.toggleFieldCentric());
    leftJoystick.button(6).onTrue(driveSubsystem.toggleFastMode());
    leftJoystick.button(7).onTrue(driveSubsystem.toggleLimitDrive());

    rightJoystick.button(1).whileTrue(checkAprilTagNumber(alignByReefRight, alignByIntake));
    rightJoystick.button(2).onTrue(driveSubsystem.resetFieldCentric());
    rightJoystick.button(3).whileTrue(driveSubsystem.getRotError());
    rightJoystick.button(3).onFalse(driveSubsystem.correctError());
    rightJoystick.button(4).whileTrue(driveSubsystem.turtle());
    rightJoystick.pov(0).whileTrue(alignRotForward);
    rightJoystick.pov(180).whileTrue(alignRotBackward);

    altJoystick.button(1).whileTrue(checkManualMode(
      elevatorSubsystem.runElevator(-0.2), 
      deliverL1));
    altJoystick.button(1).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(2).whileTrue(checkManualMode(
      Commands.print(""), 
      deliverL2));
    altJoystick.button(2).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(3).whileTrue(checkManualMode(
      elevatorSubsystem.runElevator(0.2), 
      deliverL3));
    altJoystick.button(3).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(4).whileTrue(checkManualMode(
      pivotSubsytem.setPivot(61.5), 
      reset));
    altJoystick.button(4).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(5).whileTrue(checkManualMode(
      pivotSubsytem.pivot(0.075),
      intake));
    altJoystick.button(5).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(6).whileTrue(checkManualMode(
      pivotSubsytem.pivot(-0.075), 
      deliverL4));
    altJoystick.button(6).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(8).onTrue(toggleManualMode());
    altJoystick.button(9).onTrue(climbSubsystem.toggleAttach());
    altJoystick.button(10).onTrue(climbSubsystem.toggleClimb());
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

  public Command checkAprilTagNumber(Command reefCommand, Command intakeCommand) {
    return new RunCommand(() -> {
      if (!intakeCommand.isScheduled() && Set.of(1, 2, 12, 13).contains(driveSubsystem.getAprilTagNumber())) {
        intakeCommand.schedule();
        System.out.println("intake command");
      }
      else if (!reefCommand.isScheduled() && Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22).contains(driveSubsystem.getAprilTagNumber())){
        reefCommand.schedule();
        System.out.println("reef command");
      }
    }).finallyDo(interupted -> {
      ledSubsystem.allianceColor();
      reefCommand.cancel();
      intakeCommand.cancel();
    });
  }

  public Command checkManualMode(Command manualCommand, Command autoCommand) {
    return new RunCommand(() -> {
      if (manualMode & !manualCommand.isScheduled()) {
        manualCommand.schedule();
      }
      else if (!manualMode & !autoCommand.isScheduled()){
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
