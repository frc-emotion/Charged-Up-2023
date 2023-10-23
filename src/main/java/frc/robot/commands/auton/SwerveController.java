package frc.robot.commands.auton;


import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

/**
 * Static method that starts swerve controller command for ease of use
 * TODO: Add path planner event markers
 */
public final class SwerveController {

  // public static Command followTrajectoryCommand(PathPlannerPath path, boolean isFirstPath, SwerveSubsytem swerveSubsystem) {

    // // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
    // return new FollowPathWithEvents(
    //     new FollowPathHolonomic(
    //         path,
    //         swerveSubsystem::getCurrentPose, // Robot pose supplier
    //         swerveSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         swerveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         swerveSubsystem::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //             new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //             4.5, // Max module speed, in m/s
    //             0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //             new ReplanningConfig() // Default path replanning config. See the API for the options here
    //         ),
    //         SwerveController // Reference to this subsystem to set requirements
    //     ),
    //     path, // FollowPathWithEvents also requires the path
    //     this::getPose // FollowPathWithEvents also requires the robot pose supplier
    // );
   
   /*  return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
            
            //swerveSubsystem.resetOdometry(); // before: getInitialHolonomicPose 
           }
         }),*/
         
         
         
        //  new FollowPathHolonomic(
        //      path, 
        //      swerveSubsystem::getCurrentPose, // Pose supplier
        //      swerveSubsystem::resetOdometry,
        //      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        //      xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //      yController, // Y controller (usually the same values as X controller)
        //      thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //      swerveSubsystem::setModuleStates,
        //      false, // Module states consumer
        //      swerveSubsystem // Requires this drive subsystem
        //  ),
          
      //   new InstantCommand(() -> swerveSubsystem.stopModules())
     //); 
  }
// }
