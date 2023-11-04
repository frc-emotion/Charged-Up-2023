package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GrabberAuton extends CommandBase {

    private ClawSubsystem clawSubSystem;

    private double speed;

    public GrabberAuton(ClawSubsystem claw, double speed) {
        clawSubSystem = claw;
        this.speed = speed;
        addRequirements(clawSubSystem);
        
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        clawSubSystem.setClawMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        clawSubSystem.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
