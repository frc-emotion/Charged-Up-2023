// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

// import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LevelChargeStation;
import frc.robot.commands.AutoAbstracted;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.MaceCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualControlElevator;
import frc.robot.commands.SwerveArcadeCommand;
import frc.robot.commands.SwerveXboxCommand;
import frc.robot.commands.auton.CurvedAuto;
import frc.robot.commands.auton.DriveForwardAuto;
import frc.robot.commands.auton.ExamplePathPlannerCommand;
import frc.robot.commands.auton.LevelChargeStationAuto;
import frc.robot.commands.auton.NonLevelAuto;
import frc.robot.commands.auton.PlaceAuto;
import frc.robot.commands.auton.RightTest;
import frc.robot.commands.auton.toTestAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.mace.Mace;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final SwerveSubsytem swerveSubsytem = new SwerveSubsytem();
  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  public static ElevatorSubsystem eSubsystem = new ElevatorSubsystem();
  // private final Mace mSub = new Mace(armSubsystem, eSubsystem);
  public static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static ClawSubsystem clawSubsytem = new ClawSubsystem();
  public static XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);
 
  public static AutoAbstracted autoAbstracted = new AutoAbstracted(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem);
  // public static Joystick arcadeStick = new Joystick(1);
  // public static Joystick arcadeStick2 = new Joystick(0);

  public RobotContainer() {
    swerveSubsytem.setDefaultCommand(

        /*
         * new SwerveArcadeCommand(
         * swerveSubsytem,
         * () -> arcadeStick.getRawAxis(1),
         * () -> -arcadeStick.getRawAxis(0),
         * () -> arcadeStick2.getRawAxis(2),
         * () -> !arcadeStick.getRawButton(3),
         * () -> arcadeStick.getRawButton(1),
         * () -> arcadeStick.getRawButton(2))
         */

        new SwerveXboxCommand(
            swerveSubsytem,
            () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> -driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
            () -> driverController.getLeftBumper(),
            () -> driverController.getRightBumper(),
            () -> driverController.getRightTriggerAxis(),
            () -> driverController.getLeftTriggerAxis())

    );
    configureButtonBindings();
    // clawSubsytem.setDefaultCommand(
    // new ClawCommand(clawSubsytem,
    // () -> operatorController.getYButtonPressed(),
    // () -> operatorController.getXButtonPressed(),
    // () -> operatorController.getAButtonPressed(),
    // () -> operatorController.getLeftTriggerAxis(),
    // () -> operatorController.getBButtonPressed(),
    // () -> operatorController.getRightTriggerAxis(),
    // () -> -operatorController.getLeftY(),
    // () -> -operatorController.getRightY() 
    // )

    eSubsystem.setDefaultCommand(new ManualControlElevator(eSubsystem, () -> -operatorController.getRightY()));
    armSubsystem.setDefaultCommand(new ManualArmCommand(armSubsystem, () -> -operatorController.getLeftY()));

    clawSubsytem.setDefaultCommand(
        new ClawCommand(
            clawSubsytem,
            clawSubsytem.getPieceType(),
            () -> operatorController.getLeftTriggerAxis(),
            () -> operatorController.getRightTriggerAxis(),
            () -> operatorController.getLeftBumperPressed()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(operatorController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> armSubsystem.resetPosition()));
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(new LevelChargeStation(swerveSubsytem));
    // new JoystickButton(arcadeStick, 5).onTrue(new InstantCommand(() ->
    // swerveSubsytem.zeroHeading()));
    new JoystickButton(driverController, OIConstants.kDriverZeroHeadingButtonIdx)
        .onTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));

    /* Depreciated */
    // new JoystickButton(driverController,
    // OIConstants.kDriverZeroHeadingButtonIdx).whenPressed(() ->
    // swerveSubsytem.zeroHeading());
    // new JoystickButton(arcadeStick, 5).whenPressed(() ->
    // swerveSubsytem.zeroHeading());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new ExamplePathPlannerCommand(swerveSubsytem, Robot.examplePath);
    return null;
  }

  // public Command getTaxiAuto(){
  // return new DriveForwardAuto(swerveSubsytem);
  // }

  // public Command LeftOrRightAuto(){
  // return new NonLevelAuto(swerveSubsytem);
  // }

  // public Command PlaceTaxiAuto(){
  // return new PlaceAuto(swerveSubsytem, clawSubsytem, mSub, armSubsystem,
  // eSubsystem, Robot.taxiBlue);
  // }

  // public Command RightMostForward() {
  //   return new RightTest(swerveSubsytem, Robot.rightMost);
  // }

  public Command LevelChargeStation() {
    return new LevelChargeStationAuto(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem, Robot.levelcenter);
  }

  // public Command CurvedTest() {
  //   return new CurvedAuto(swerveSubsytem, Robot.placeauto);
  // }

  public Command EasyToTest(PathPlannerTrajectory pathToUse, Double timeoutTime) {
    return new toTestAuto(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem, pathToUse, timeoutTime);
  }

}
