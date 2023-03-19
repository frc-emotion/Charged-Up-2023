package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem arm;
    private Supplier<Double> angularSpdFunc;

    public ManualArmCommand(ArmSubsystem arm,  Supplier<Double> angularSpdFunc){
        this.arm = arm;
        this.angularSpdFunc = angularSpdFunc;

        addRequirements(arm);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double angularSpeed = angularSpdFunc.get();
            arm.stopArm();


<<<<<<< HEAD
=======
        if(angularSpeed > OIConstants.kDeadband){
            arm.setArmSpeeds(ArmConstants.ARM_SPEED);
        }
        else if(angularSpeed < -OIConstants.kDeadband){
            arm.setArmSpeeds(-ArmConstants.ARM_SPEED);
        }
        else{
            arm.stopArm();
        }
>>>>>>> e7ba19096697bc42d72de0379a9c13dba750d47a
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}