package frc.robot.util.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

public class LimeLightAlign extends CommandBase {

    private final SwerveSubsytem swerveSubsystem;
    private final LimeLight limelight;
    private ChassisSpeeds alignmentSpeeds;
    private final PIDController alignmentController;
    private double turningSpeed;

    public LimeLightAlign(SwerveSubsytem swerveSubsystem, LimeLight limelight){
        this.limelight = limelight;
        this.swerveSubsystem = swerveSubsystem;

        alignmentController = new PIDController(Constants.DriveConstants.kPAlignment, Constants.DriveConstants.kIAlignment, Constants.DriveConstants.kDAlignment);

        alignmentSpeeds = new ChassisSpeeds();

        addRequirements(swerveSubsystem);
        addRequirements(limelight);
        

    }

    @Override
    public void initialize(){
        alignmentController.setSetpoint(0);
        alignmentController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        turningSpeed = alignmentController.calculate(Units.degreesToRadians(limelight.getEntry("tx")));
        alignmentSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
        swerveSubsystem.setChassisSpeeds(alignmentSpeeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(alignmentSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        System.out.println(turningSpeed);

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
