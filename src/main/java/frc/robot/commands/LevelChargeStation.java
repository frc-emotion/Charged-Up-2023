package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LevelChargeStation extends CommandBase{

    private final SwerveSubsytem swerve;

    private boolean inControl;

    private static final TrapezoidProfile.Constraints travelConstraints = new TrapezoidProfile.Constraints(DriveConstants.MAX_LEVEL_VELOCITY, DriveConstants.MAX_LEVEL_ACCELERATION);
    private final ProfiledPIDController angleController = new ProfiledPIDController(SmartDashboard.getNumber("KP Constant", DriveConstants.KPLevel), SmartDashboard.getNumber("KD Constant", DriveConstants.KDLevel), SmartDashboard.getNumber("KI Constant", DriveConstants.KILevel), travelConstraints); //General approximation should have Kp be something like Max Velocity / max Angle

    public LevelChargeStation(SwerveSubsytem swerve){

        this.swerve = swerve;

        inControl = false; // Used to account for if angle is negative considering that PID calculate determines actual sign of velocity
        angleController.setTolerance(Math.PI/180); 
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
        SmartDashboard.putNumber("KP Constant", DriveConstants.KPLevel);
        SmartDashboard.putNumber("KD Constant", DriveConstants.KDLevel);
        SmartDashboard.putNumber("KI Constant", DriveConstants.KILevel);
    }
    
    @Override
    public void initialize() {
        
        angleController.setGoal(DriveConstants.TARGET_ANGLE); // In this version which only uses one PID loop goal is set immediately since it does not change 
    }

    @Override
    public void execute() {
        var stationHeadingAngle = Units.degreesToRadians(swerve.getHeading());
        double error1 = (-swerve.getPitch() * Math.cos(stationHeadingAngle)) - (swerve.getRoll() * Math.sin(stationHeadingAngle)); // Returns measure of Roll and Pitch and takes cos and sin to determine error
        var levelSpeed = 0.0;
        
        //Checks to see if the Robot is on the Charge Station and tilted 
        if (Math.abs(error1) > DriveConstants.THRESHOLD || inControl){
            if (!inControl){
                inControl = true;
            }
            levelSpeed = angleController.calculate(error1);
        }

        ChassisSpeeds speeds = new ChassisSpeeds((levelSpeed * Math.cos(stationHeadingAngle)), (levelSpeed * Math.sin(stationHeadingAngle)), 0); //Speed is always in the correct heading assuming that heading angle is constant throughout the field
        swerve.setChassisSpeeds(speeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        Rotation2d leftRotation = new Rotation2d(Math.PI/4);
        Rotation2d rightRotation = new Rotation2d(-Math.PI/4);

        SwerveModuleState[] stoppedStates = {new SwerveModuleState(0, leftRotation)};
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(DriveConstants.TARGET_ANGLE - (-swerve.getPitch() * Math.cos(Units.degreesToRadians(swerve.getHeading())) - (swerve.getRoll() * Math.sin(Units.degreesToRadians(swerve.getHeading()))))) < DriveConstants.THRESHOLD; //Maybe have it end the command when the specified threshold is met and the robot is 'flat'?
        //return false;
    }
}
