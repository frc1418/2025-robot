// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AlignByFieldPose;
import frc.robot.commands.DeliverL1;
import frc.robot.commands.DeliverL2;
import frc.robot.commands.DeliverL3;
import frc.robot.commands.DeliverL4;
import frc.robot.commands.DropFromL4;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.L4Auto;
import frc.robot.commands.Reset;
import frc.robot.commands.ResetAuto;
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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {

  private enum ControllerMode {
    AUTO("Auto"),
    MANUAL("Manual"),
    ADJUSTMENT("Adjustmsent");

    private final String name;
    ControllerMode(String name) {
      this.name = name;
    }

    public String getName() {
      return name;
    }
  }
  
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

  private final AlignByFieldPose alignByReefLeft = new AlignByFieldPose(driveSubsystem, ledSubsystem, -0.22, 0.54, 0, 0.5, 0, 0.1, 0, 3);
  private final AlignByFieldPose alignByReefRight = new AlignByFieldPose(driveSubsystem, ledSubsystem, 0.11, 0.54 , 0, 0.5, 0, 0.1, 0, 3);
  private final AlignByFieldPose alignByIntake = new AlignByFieldPose(driveSubsystem, ledSubsystem, 0, 0.685, 0, 0.5, 0, 0.1, 0, 3);
  private final AlignRot alignRotBackward = new AlignRot(this, driveSubsystem, leftJoystick, 180);
  private final AlignRot alignRotForward = new AlignRot(this, driveSubsystem, leftJoystick, 0);

  private final L4Auto l4Auto;
  private final DropFromL4 dropFromL4;
  private final DeliverL4 deliverL4;
  private final DeliverL3 deliverL3;
  private final DeliverL2 deliverL2;
  private final DeliverL1 deliverL1;
  private final Intake intake;
  private final IntakeAuto intakeAuto;
  private final Reset reset;
  private final ResetAuto resetAuto;

  private enum AlignmentType {
    LEFT_DELIVERY("Left Delivery"),
    RIGHT_DELIVERY("Right Delivery"),
    INTAKE("Intake");

    private final String name;

    AlignmentType(String name) {
      this.name = name;
    }

    public String getName() {
      return name;
    }
  }

  private ControllerMode controllerMode = ControllerMode.AUTO;
  private AlignmentType alignmentType = AlignmentType.INTAKE;
  
  public RobotContainer(RobotBase robot) {
    l4Auto = new L4Auto(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    dropFromL4 = new DropFromL4(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL4 = new DeliverL4(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL3 = new DeliverL3(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL2 = new DeliverL2(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    deliverL1 = new DeliverL1(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    intake = new Intake(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    intakeAuto = new IntakeAuto(pivotSubsytem, elevatorSubsystem, intakeSubsystem, ledSubsystem, robot);
    reset = new Reset(pivotSubsytem, elevatorSubsystem, ledSubsystem, robot);
    resetAuto = new ResetAuto(pivotSubsytem, elevatorSubsystem, ledSubsystem, robot);

    NamedCommands.registerCommand("elevatorIntake", elevatorSubsystem.moveElevatorToHeight(0.222));
    NamedCommands.registerCommand("elevatorDeliver", elevatorSubsystem.moveElevatorToHeight(1.01));
    NamedCommands.registerCommand("pivotIntake", pivotSubsytem.setPivot(36));
    NamedCommands.registerCommand("pivotDeliver", pivotSubsytem.setPivot(-31));
    NamedCommands.registerCommand("intakeIn", intakeSubsystem.intakeIn());
    NamedCommands.registerCommand("intakeOut", intakeSubsystem.intakeOut());
    NamedCommands.registerCommand("deliverL1", deliverL1);
    NamedCommands.registerCommand("deliverL2", deliverL2);
    NamedCommands.registerCommand("deliverL3", deliverL3);
    NamedCommands.registerCommand("deliverL4", l4Auto);
    NamedCommands.registerCommand("resetL4", dropFromL4);
    NamedCommands.registerCommand("intake", intakeAuto);
    NamedCommands.registerCommand("reset", resetAuto);
    NamedCommands.registerCommand("zeroElevator", elevatorSubsystem.reZero());

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

    altJoystick.button(1).whileTrue(checkMode(
      elevatorSubsystem.runElevator(-0.2), 
      pivotSubsytem.zeroPivot(), 
      Commands.none()));
    altJoystick.button(1).onTrue(Commands.runOnce(() -> {
      if (ControllerMode.ADJUSTMENT.equals(controllerMode)) {
        adjustBackAlignment(0.01);
      }
    }));
    altJoystick.button(2).whileTrue(checkMode(
      Commands.none(), 
      deliverL3, 
      Commands.none()));
    altJoystick.button(2).onTrue(Commands.runOnce(() -> {
      if (ControllerMode.ADJUSTMENT.equals(controllerMode)) {
        adjustSideOffset(0.01);
      }
    }));
    altJoystick.button(2).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(3).whileTrue(checkMode(
      elevatorSubsystem.runElevator(0.2), 
      deliverL2, 
      Commands.none()));
    altJoystick.button(3).onTrue(Commands.runOnce(() -> {
      if (ControllerMode.ADJUSTMENT.equals(controllerMode)) {
        adjustSideOffset(-0.01);
      }
    }));
    altJoystick.button(3).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(4).whileTrue(checkMode(
      Commands.none(), 
      reset, 
      Commands.none()));
    altJoystick.button(4).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(4).onTrue(Commands.runOnce(() -> {
      if (ControllerMode.ADJUSTMENT.equals(controllerMode)) {
        adjustBackAlignment(-0.01);
      }
    }));
    altJoystick.button(5).whileTrue(checkMode(
      pivotSubsytem.pivot(0.075),
      intake,
      Commands.none()));
    altJoystick.button(5).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(6).whileTrue(checkMode(
      pivotSubsytem.pivot(-0.075),
      deliverL4,
      Commands.none()));
    altJoystick.button(6).onFalse(ledSubsystem.allianceColor());
    altJoystick.button(8).onTrue(toggleManualMode());
    altJoystick.button(9).onTrue(Commands.runOnce(() -> {
        climbSubsystem.attachToggle();
      }));
    altJoystick.button(10).onTrue(Commands.runOnce(() -> {
        climbSubsystem.climbToggle();
      }));
    altJoystick.pov(0).whileTrue(checkMode(intakeSubsystem.intakeOut(), intakeSubsystem.intakeOut(), Commands.none()));
    altJoystick.pov(90).onTrue(toggleAlignmentMode(AlignmentType.RIGHT_DELIVERY));
    altJoystick.pov(180).whileTrue(checkMode(intakeSubsystem.intakeIn(), Commands.none(), Commands.none()));
    altJoystick.pov(180).onTrue(toggleAlignmentMode(AlignmentType.INTAKE));
    altJoystick.pov(270).onTrue(toggleAlignmentMode(AlignmentType.LEFT_DELIVERY));
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

  public Command toggleAlignmentMode(AlignmentType alignmentType) {
    return Commands.runOnce(() -> {
      if (ControllerMode.ADJUSTMENT.equals(controllerMode)) {
        controllerMode = ControllerMode.AUTO;
      }
      else {
        controllerMode = ControllerMode.ADJUSTMENT;
        this.alignmentType = alignmentType;
      }
      System.out.println("Controller Mode: " + controllerMode.getName());
      this.alignmentType = alignmentType;
      System.out.println("Alignment Type: " + alignmentType.getName());
    }); 
  }

  public Command toggleManualMode() {
    return Commands.runOnce(() -> {
      if (ControllerMode.AUTO.equals(controllerMode) || ControllerMode.ADJUSTMENT.equals(controllerMode)) {
        controllerMode = ControllerMode.MANUAL;
      }
      else {
        controllerMode = ControllerMode.AUTO;
      }
      System.out.println("Controller Mode: " + controllerMode.getName());
    });
  }

  public Command checkAprilTagNumber(Command reefCommand, Command intakeCommand) {
    return new RunCommand(() -> {
      if (!intakeCommand.isScheduled() && Set.of(1, 2, 12, 13).contains(driveSubsystem.getAprilTagNumber())) {
        intakeCommand.schedule();
      }
      else if (!reefCommand.isScheduled() && Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22).contains(driveSubsystem.getAprilTagNumber())){
        reefCommand.schedule();
      }
    }).finallyDo(interupted -> {
      ledSubsystem.allianceColor();
      reefCommand.cancel();
      intakeCommand.cancel();
    });
  }

  public void adjustSideOffset(double offset) {
    switch (alignmentType) {
      case LEFT_DELIVERY:
      alignByReefLeft.adjustSideOffset(offset, alignmentType.getName());
      break;
      case RIGHT_DELIVERY:
      alignByReefRight.adjustSideOffset(offset, alignmentType.getName());
      break;
      case INTAKE:
      alignByIntake.adjustSideOffset(offset, alignmentType.getName());
      break;
    }
  }

  public void adjustBackAlignment(double offset) {
    switch (alignmentType) {
      case LEFT_DELIVERY:
      alignByReefLeft.adjustBackOffset(offset, alignmentType.getName());
      break;
      case RIGHT_DELIVERY:
      alignByReefRight.adjustBackOffset(offset, alignmentType.getName());
      break;
      case INTAKE:
      alignByIntake.adjustBackOffset(offset, alignmentType.getName());
      break;
    }  
  }


  public Command checkMode(Command manualCommand, Command autoCommand, Command adjustmentCommand) {
    return new RunCommand(() -> {
      if (ControllerMode.MANUAL.equals(controllerMode) && !manualCommand.isScheduled()) {
        manualCommand.schedule();
      }
      else if (ControllerMode.AUTO.equals(controllerMode) && !autoCommand.isScheduled()){
        autoCommand.schedule();
      }
      else if (ControllerMode.ADJUSTMENT.equals(controllerMode) && !adjustmentCommand.isScheduled()){
        adjustmentCommand.schedule();
      }
    }).finallyDo(interupted -> {
      manualCommand.cancel();
      autoCommand.cancel();
      adjustmentCommand.cancel();
    });
  }

  public void resetPivot() {
    pivotSubsytem.zero();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
