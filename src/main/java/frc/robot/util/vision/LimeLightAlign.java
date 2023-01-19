package frc.robot.util.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.util.vision.LimeLight;

public class LimeLightAlign extends CommandBase {

    private final SwerveSubsytem swerveSubsystem;
    private final LimeLight limelight;
    private ChassisSpeeds alignmentSpeeds;

    public LimeLightAlign(SwerveSubsytem swerveSubsystem, LimeLight limelight){
        this.limelight = limelight;
        this.swerveSubsystem = swerveSubsystem;

        alignmentSpeeds = new ChassisSpeeds();

        addRequirements(swerveSubsystem);
        addRequirements(limelight);
        

    }

    @Override
    public void execute() {
        alignmentSpeeds = new ChassisSpeeds(0, 0, limelight.getTurningSpeed());
        swerveSubsystem.setChassisSpeeds(alignmentSpeeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(alignmentSpeeds);
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
