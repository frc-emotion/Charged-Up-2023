package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;


public class LevelChargeStation extends CommandBase{

    private final SwerveSubsytem swerve;

    private boolean inControl;

    private static final TrapezoidProfile.Constraints travelConstraints = new TrapezoidProfile.Constraints(DriveConstants.MAX_LEVEL_VELOCITY, DriveConstants.MAX_LEVEL_ACCELERATION);
    private final ProfiledPIDController angleController = new ProfiledPIDController(DriveConstants.KPLevel, DriveConstants.KDLevel, DriveConstants.KILevel, travelConstraints); //General approximation should have Kp be something like Max Velocity / max Angle

    public LevelChargeStation(SwerveSubsytem swerve){

        this.swerve = swerve;

        inControl = false; // Used to account for if angle is negative considering that calculate determines actual sign of velocity
        angleController.setTolerance(Math.PI/180); //Not sure if radian conversion is needed here considering everything else is done with degrees in mind
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        angleController.setGoal(DriveConstants.TARGET_ANGLE); // In this version which only uses one PID loop goal is set immediately since it does not change 
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        double alpha = swerve.getPitch(); // Returns angle measure from -pi to pi
        //double beta = swerve.getRoll();
        var robotPose2d = swerve.getCurrentPose();
        var ySpeed = 0.0;
        //var xSpeed = 0.0;
=======
        var stationHeadingAngle = swerve.getAlignmentHeading() - DriveConstants.REFERENCE_HEADING;
        double error1 = (swerve.getPitch() * Math.cos(stationHeadingAngle)) + (swerve.getRoll() * Math.sin(stationHeadingAngle)); // Returns angle measure from -180 to 180 ( Might need to change to a - instead of a + depending on if I interpreted roll correctly)
        var levelSpeed = 0.0;
>>>>>>> 861dc7bf4156bd82ac18aaeb06c804899bdedeaa
        
        //Checks to see if the Robot is on the Charge Station and tilted 
        if (Math.abs(error1) > DriveConstants.THRESHOLD || inControl){
            if (!inControl){
                inControl = true;
            }
<<<<<<< HEAD
            ySpeed = angleController.calculate(-alpha);
                     
=======
            levelSpeed = angleController.calculate(error1);
>>>>>>> 861dc7bf4156bd82ac18aaeb06c804899bdedeaa
        }

        ChassisSpeeds speeds = new ChassisSpeeds((levelSpeed * Math.cos(stationHeadingAngle)), (levelSpeed * Math.sin(stationHeadingAngle)), 0);
        swerve.setChassisSpeeds(speeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        //return Math.abs(DriveConstants.TARGET_ANGLE - gyro.getPitch()) < DriveConstants.THRESHOLD; //Maybe have it end the command when the specified threshold is met and the robot is 'flat'?
        return false;
    }
}
