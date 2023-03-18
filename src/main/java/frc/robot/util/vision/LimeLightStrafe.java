

package frc.robot.util.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

public class LimeLightStrafe extends CommandBase{
   /*  private final SwerveSubsytem swerveSubsystem;
    private final LimeLight limelight;
    private ChassisSpeeds strafeSpeeds, turnSpeeds;

    private final PIDController vxController, vyController, angController;
    private double xSpeed, ySpeed, rotSpeed;
    private double tx, ty, heading;
    

    public LimeLightStrafe(SwerveSubsytem swerveSubsystem, LimeLight limelight){
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
        angController.setSetpoint(0); //not sure about this setpoint
        angController.enableContinuousInput(-Math.PI, Math.PI);

        //not sure how big tolerances are supposed to be
    }

    @Override
    public void execute() {

        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        heading = swerveSubsystem.getCurrentPose().getRotation().getRadians();

        xSpeed = vxController.calculate(limelight.getLateral(tx, ty));
        ySpeed = vyController.calculate(limelight.getDistance(tx, ty));
        rotSpeed = angController.calculate(heading);

        print();

        turnSpeeds = new ChassisSpeeds(0, 0, rotSpeed);
        strafeSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
        

        if(Math.abs(heading) > Units.degreesToRadians(1)){
            swerveSubsystem.setChassisSpeeds(turnSpeeds);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(turnSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);

        }
        else {
            swerveSubsystem.setChassisSpeeds(strafeSpeeds);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(strafeSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
        }

    }

    @Override 
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void print(){
        System.out.println("height: " + limelight.getHeight());
        System.out.println("hypotenuse: " + limelight.getHypotenuse(limelight.getEntry("ty")));
        System.out.println("x distance: " + limelight.getLateral(limelight.getEntry("tx"), limelight.getEntry("ty")));
        System.out.println("y distance: " + limelight.getDistance(limelight.getEntry("tx"), limelight.getEntry("ty")));
        System.out.println("heading: " + heading);
        System.out.println("rotSpeed: " + rotSpeed);
        System.out.println("xSpeed: " + xSpeed);
        System.out.println("ySpeed: " + ySpeed);
        System.out.println("------------");
    }
 */

}

