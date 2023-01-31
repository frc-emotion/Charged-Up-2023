package frc.robot.util.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

public class LimeLightStrafe extends CommandBase{
    private final SwerveSubsytem swerveSubsystem;
    private final LimeLight limelight;
    private ChassisSpeeds strafeSpeeds;
    private final PIDController vxController, vyController, angController;
    private double xSpeed, ySpeed, rotSpeed;

    public LimeLightStrafe(SwerveSubsytem swerveSubsystem, LimeLight limelight){
        this.limelight = limelight;
        this.swerveSubsystem = swerveSubsystem;

        strafeSpeeds = new ChassisSpeeds();

        vxController = new PIDController(Constants.DriveConstants.kPAlignment, Constants.DriveConstants.kIAlignment, Constants.DriveConstants.kDAlignment);
        vyController = new PIDController(Constants.DriveConstants.kPAlignment, Constants.DriveConstants.kIAlignment, Constants.DriveConstants.kDAlignment);
        angController = new PIDController(Constants.DriveConstants.kPAlignment, Constants.DriveConstants.kIAlignment, Constants.DriveConstants.kDAlignment);

        addRequirements(swerveSubsystem);
        addRequirements(limelight);
    }

    @Override
    public void initialize(){
        vxController.setSetpoint(0);
        vyController.setSetpoint(0);
        angController.setSetpoint(0); //not sure about this setpoint
        angController.enableContinuousInput(-Math.PI, Math.PI);

        //not sure how big tolerances are supposed to be
    }

    @Override
    public void execute() {

        xSpeed = vxController.calculate(limelight.getLateral(limelight.getEntry("tx"), limelight.getEntry("ty")));
        ySpeed = vyController.calculate(limelight.getDistance(limelight.getEntry("ty")));
        rotSpeed = angController.calculate(swerveSubsystem.getCurrentPose().getRotation().getRadians());

        strafeSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        swerveSubsystem.setChassisSpeeds(strafeSpeeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(strafeSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override 
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
