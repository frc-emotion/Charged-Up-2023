// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualControlElevator;
import frc.robot.commands.SwerveXboxCommand;
import frc.robot.commands.auton.Autonomous;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;

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
  public static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static ClawSubsystem clawSubsytem = new ClawSubsystem();
  public static XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);
 
  // public static Joystick arcadeStick = new Joystick(1);
  // public static Joystick arcadeStick2 = new Joystick(0);

  public RobotContainer() {
    swerveSubsytem.setDefaultCommand(

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

    new JoystickButton(driverController, OIConstants.kDriverZeroHeadingButtonIdx)
        .onTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));


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

  // Returns the command based on path & timeout given
  // Overload incase additional commands wanted

  public Command EasyToUse(PathPlannerTrajectory pathToUse, double timeoutTime, boolean place) {
    //swerveSubsytem.autoGyro();
    return new Autonomous(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem, pathToUse, timeoutTime, place, false);
  }

  public Command EasyToUse(PathPlannerTrajectory pathToUse, double timeoutTime, boolean place, Command additional) {
    //swerveSubsytem.autoGyro();
    return new Autonomous(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem, pathToUse, timeoutTime, place, false, additional);
  }

  public void setRobotSpeedDivisor(double divisor) {
    swerveSubsytem.setSpeedType(divisor);
  }

  public Command TestAuto(PathPlannerTrajectory pathToUse, double timeoutTime, boolean place) {
    return new Autonomous(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem, pathToUse, timeoutTime, place, true);
  }

  public Command TestAuto(PathPlannerTrajectory pathToUse, double timeoutTime, boolean place, Command additional) {
    return new Autonomous(swerveSubsytem, armSubsystem, eSubsystem, clawSubsytem, pathToUse, timeoutTime, place, true, additional);
  }



}
