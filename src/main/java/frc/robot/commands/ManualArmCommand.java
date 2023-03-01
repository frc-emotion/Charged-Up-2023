package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final Supplier<Double> angularSpdFunc;

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

        if(angularSpeed > 0.0){
            arm.setArmSpeeds(ArmConstants.ARM_SPEED);
        }
        else if(angularSpeed < 0.0){
            arm.setArmSpeeds(-ArmConstants.ARM_SPEED);
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