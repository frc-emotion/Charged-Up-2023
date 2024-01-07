package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class GrabberAuton extends CommandBase {

    private ClawSubsystem clawSubSystem;

    private double speed;

    public GrabberAuton(ClawSubsystem claw, double speed) {
        clawSubSystem = claw;
        this.speed = speed;
        addRequirements(clawSubSystem);
        
    }

    public static GrabberAuton getGrabberAuto(double speed) {
        return new GrabberAuton(new ClawSubsystem(), speed);
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
