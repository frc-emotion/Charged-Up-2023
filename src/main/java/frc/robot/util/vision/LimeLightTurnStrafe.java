

package frc.robot.util.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

public class LimeLightTurnStrafe extends CommandBase{
   /*  private final SwerveSubsytem swerveSubsystem;
    private final LimeLight limelight;
    private ChassisSpeeds strafeSpeeds;

    private final PIDController vxController, vyController, angController;
    private double xSpeed, ySpeed, rotSpeed;
    private double tx, ty, heading;
    

    public LimeLightTurnStrafe(SwerveSubsytem swerveSubsystem, LimeLight limelight){
        this.limelight = limelight;
        this.swerveSubsystem = swerveSubsystem;

        strafeSpeeds = new ChassisSpeeds();

        vxController = new PIDController(Constants.DriveConstants.kPDrive, Constants.DriveConstants.kIDrive, Constants.DriveConstants.kDDrive);
        vyController = new PIDController(Constants.DriveConstants.kPDrive, Constants.DriveConstants.kIDrive, Constants.DriveConstants.kDDrive);
        angController = new PIDController(Constants.DriveConstants.kPAlignment, Constants.DriveConstants.kIAlignment, Constants.DriveConstants.kDAlignment);

        addRequirements(swerveSubsystem);
        addRequirements(limelight);
    }

    @Override
    public void initialize(){
        vxController.setSetpoint(0);
        vyController.setSetpoint(0);
        angController.setSetpoint(0); 
        angController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {

        heading = swerveSubsystem.getCurrentPose().getRotation().getRadians();
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");

        xSpeed = vxController.calculate(limelight.getXDist(tx, ty, heading));
        ySpeed = vyController.calculate(limelight.getYDist(tx, ty, heading));
        rotSpeed = angController.calculate(heading);

        strafeSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        strafeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(strafeSpeeds, Rotation2d.fromDegrees(heading));
        
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
*/

}

