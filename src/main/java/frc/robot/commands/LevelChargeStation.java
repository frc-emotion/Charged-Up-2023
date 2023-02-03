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
        double alpha = swerve.getPitch(); // Returns angle measure from -pi to pi
        var robotPose2d = swerve.getCurrentPose();
        var ySpeed = 0.0;
        
        //Checks to see if the Robot is on the Charge Station and tilted 
        if (-alpha > DriveConstants.THRESHOLD || inControl){
            if (!inControl){
                inControl = true;
            }
            ySpeed = angleController.calculate(-alpha);           
        }

        ChassisSpeeds speeds = new ChassisSpeeds(ySpeed, 0, 0);
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