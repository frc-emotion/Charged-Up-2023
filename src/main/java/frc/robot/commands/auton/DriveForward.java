package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;


public class DriveForward extends CommandBase {

    public final SwerveSubsytem swerveSubsystem;
    private double speed;
    
    public DriveForward(SwerveSubsytem swerveSubsytem, double speed) {

        this.swerveSubsystem = swerveSubsytem;
        this.speed = speed;
        
        addRequirements(swerveSubsystem);

        // set chasis speeds set in xbox command

        // make method that runs 4 drive motors forward

        // have a second ounter

        // parameter in command to adjust speed (power)

        
    }

    @Override
    public void execute() {
        setSpeed(new ChassisSpeeds(speed, 0, 0));
    }

    public void setSpeed(ChassisSpeeds speedToSet) {
    
        swerveSubsystem.driveRobotRelative(speedToSet);
    }

    

}
