package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MacroArmCommand extends CommandBase {

    private ArmSubsystem arm;
    private Supplier<Double> angularSpdFunc;

    public MacroArmCommand(ArmSubsystem arm){
        this.arm = arm;


        addRequirements(arm);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() { // Don't use needs way more work 
        arm.setArmAngle();
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