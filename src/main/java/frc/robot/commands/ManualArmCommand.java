package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.mace.Mace;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem arm;
    private Supplier<Double> angularSpdFunc;

    public ManualArmCommand(ArmSubsystem arm, Supplier<Double> angularSpdFunc){
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



        if(angularSpeed > OIConstants.kDeadband){
            arm.setArmSpeeds(ArmConstants.ARM_SPEED);
        }
        else if(angularSpeed < -OIConstants.kDeadband){
            arm.setArmSpeeds(-ArmConstants.ARM_SPEED);
        } else if(RobotContainer.operatorController.getBackButton()){
            arm.setArmSpeeds(arm.getPIDOutputVolts(Units.degreesToRadians(-30)));
        } else if(RobotContainer.operatorController.getBButton()){
            arm.setArmSpeeds(arm.getPIDOutputVolts(Units.degreesToRadians(180)));
        } else if (RobotContainer.operatorController.getLeftTriggerAxis() > 0.1){
            arm.setArmSpeeds(arm.getPIDOutputVolts(Units.degreesToRadians(270)));
        }
        else{
            arm.stopArm();
        }
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