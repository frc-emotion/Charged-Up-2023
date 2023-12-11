// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.auton.LockWheels;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

public static PathPlannerTrajectory  examplePath, rightMost, levelcenter, placeauto, bottomCurved, straightBottomCone, smoothCurveTop, topStraight;
public static Command examplePathCommand, taxiBlueCommand, levelCenterCommand;

  XboxController controller2 = new XboxController(2);


  private SendableChooser<Integer> autoDropdown = new SendableChooser<>();

  private SendableChooser<Integer> speedDropdown = new SendableChooser<>();

  //private SendableChooser<Integer> allianceChooser = new SendableChooser<>();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   // Load paths here

    rightMost = PathPlanner.loadPath("Forward", 2, 1); // 2 1

    levelcenter = PathPlanner.loadPath("Levelforward", 0.526, 0.5); // 0.5 0.5 

    placeauto = PathPlanner.loadPath("PlaceAuto", 2, 1);
    
    bottomCurved = PathPlanner.loadPath("BestCurved", 2, 1);

    straightBottomCone = PathPlanner.loadPath("DirectPickupB", 2, 1);

    smoothCurveTop = PathPlanner.loadPath("TopCurveCone", 2, 1);
    
    topStraight = PathPlanner.loadPath("TopStraight2Cone", 2, 1);

    m_robotContainer = new RobotContainer();

    autoDropdown.setDefaultOption("Level", 1);
    autoDropdown.addOption("Rightmost Forward Test", 2);
    autoDropdown.addOption("Curved Auto Test", 3);

    autoDropdown.addOption("Bottom Curved Test", 4);
    autoDropdown.addOption("Straight To Cone Bottom Test", 5);
    autoDropdown.addOption("Smooth Curved Path (Top)", 6);
    autoDropdown.addOption("Straight Path (Top)", 7);

    SmartDashboard.putData("Auto Path?", autoDropdown);

    speedDropdown.addOption("Slow Speed", 1);
    speedDropdown.addOption("Impacted Speed", 2);
    speedDropdown.setDefaultOption("Regular speed", 3);
    speedDropdown.addOption("Max Speed", 4);

    SmartDashboard.putData("Driving Speed", autoDropdown);

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    switch (speedDropdown.getSelected()) {
      case 1: // Slowest Speed
        m_robotContainer.setRobotSpeedDivisor(6.0);
        break;

      case 2: // Slow Speed
        m_robotContainer.setRobotSpeedDivisor(4.0);
        break;

      case 3: // Regular Speed
        m_robotContainer.setRobotSpeedDivisor(2.0);
        break;

      case 4: // Max Speed
        m_robotContainer.setRobotSpeedDivisor(1.0);
        break;

    }
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  public void autonomousInit() {
    switch (autoDropdown.getSelected()) {
      // Essentially just one method for any path which is why its easy to use
      // Take timeout estimates from pathplanner estimates (Make sure correct velocity and accel are set)

      case 1:
        m_autonomousCommand = m_robotContainer.EasyToUse(
          levelcenter, 
          5.5,
          true,
          new LockWheels(RobotContainer.swerveSubsytem).withTimeout(2) // Extra command for special cases such as leveling
          );
        break;
        
      case 2:
        m_autonomousCommand = m_robotContainer.EasyToUse(rightMost, 8.5, false);
        break;

      case 3:
        m_autonomousCommand = m_robotContainer.EasyToUse(placeauto, 16.5, false);
        break;

      case 4:
        m_autonomousCommand = m_robotContainer.EasyToUse(bottomCurved, 10.5, false);
        break;

      case 5:
        m_autonomousCommand = m_robotContainer.EasyToUse(straightBottomCone, 10.5, false);
        break;

      case 6:
        m_autonomousCommand = m_robotContainer.EasyToUse(smoothCurveTop, 12.5, false);
        break;

      case 7:
        m_autonomousCommand = m_robotContainer.EasyToUse(topStraight, 5.5, false);
        break;

      default:
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        break;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
