package frc.robot.util.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsytem;

public class LimeLight extends SubsystemBase{

    ChassisSpeeds alignmentSpeeds;
    private final SwerveSubsytem swerveSubsystem;
    private final PIDController alignmentController;
    private double turningSpeed;
    private String table; 


    public LimeLight(){
        table = "limelight";
        alignmentController = new PIDController(Constants.DriveConstants.kPAlignment, Constants.DriveConstants.kIAlignment, Constants.DriveConstants.kDAlignment);
        swerveSubsystem = new SwerveSubsytem();
    }

    private double getEntry(String selector) {
        return NetworkTableInstance.getDefault().getTable(table).getEntry(selector).getDouble(0);
    }

    
    public double getTurningSpeed(){
        turningSpeed = alignmentController.calculate(getEntry("tx"), 0);
        return turningSpeed;
    }
   


}
