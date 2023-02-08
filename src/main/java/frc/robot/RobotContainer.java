// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LevelChargeStation;
import frc.robot.commands.SwerveArcadeCommand;
import frc.robot.commands.SwerveXboxCommand;
import frc.robot.commands.auton.ExamplePathPlannerCommand;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.util.vision.PoseEstimator;
import com.kauailabs.navx.frc.AHRS;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsytem swerveSubsytem = new SwerveSubsytem();
  public static  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController operatorController = new XboxController(1);
  //public static Joystick arcadeStick = new Joystick(1);
  public static Joystick arcadeStick2 = new Joystick(0);
  public static AHRS gyro = new AHRS();
  private final PhotonCamera cam  = new PhotonCamera("cameraNameHere"); //FIX change camera name to what it is in Photon UI
  private final PoseEstimator poseEstimator = new PoseEstimator(cam); 


  public RobotContainer() {
    swerveSubsytem.setDefaultCommand(
    
     /* new SwerveArcadeCommand(
        swerveSubsytem, 
        () -> arcadeStick.getRawAxis(1),
        () ->  -arcadeStick.getRawAxis(0), 
        () -> arcadeStick2.getRawAxis(2), 
        () -> !arcadeStick.getRawButton(3),
        () -> arcadeStick.getRawButton(1),
        () -> arcadeStick.getRawButton(2))
`/* */
    
    new SwerveXboxCommand(
    swerveSubsytem, 
    () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
    () ->  -driverController.getRawAxis(OIConstants.kDriverXAxis), 
    () -> driverController.getRawAxis(OIConstants.kDriverRotAxis), 
    () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
    () -> driverController.getLeftBumper(),
    () -> driverController.getRightBumper())
    
    );
    configureButtonBindings(); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(arcadeStick, 5).onTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));
    new JoystickButton(driverController, XboxController.Button.kY.value).whileTrue(new LevelChargeStation(swerveSubsytem));


    new JoystickButton(driverController, OIConstants.kDriverZeroHeadingButtonIdx).onTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));

    /*Depreciated */
    //new JoystickButton(driverController, OIConstants.kDriverZeroHeadingButtonIdx).whenPressed(() -> swerveSubsytem.zeroHeading());
    //new JoystickButton(arcadeStick, 5).whenPressed(() -> swerveSubsytem.zeroHeading());

    //FIX add button for align to target 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ExamplePathPlannerCommand(swerveSubsytem, Robot.examplePath);
  }
}
