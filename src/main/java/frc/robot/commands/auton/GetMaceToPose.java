
package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GetMaceToPose extends CommandBase {

    private final ArmSubsystem aSub;
    private final ElevatorSubsystem eSub;

    boolean arm, ele, up;

    public GetMaceToPose(ArmSubsystem aSub, ElevatorSubsystem eSub, boolean arm, boolean ele, boolean up) {
        this.aSub = aSub;
        this.eSub = eSub;
        this.arm = arm;
        this.ele = ele;
        this.up = up;

        addRequirements(eSub);
    }

    @Override
    public void initialize() {
        // eSub.setElevatorGoal(0.6);
    }

    @Override
    public void execute() {

        // if (ele){
        eSub.setElevatorVoltage(0.30);
        // }

        // if(arm){
        // aSub.setArmSpeeds(up ? 0.15 : -0.15);
        // }

    }

    @Override
    public void end(boolean interrupted) {
        // eSub.stopElevator();
        // aSub.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
