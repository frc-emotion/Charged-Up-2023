package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    
    private Intake intake;

    public RunIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }


    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        if(RobotContainer.operatorController.getLeftBumperPressed()){
            intake.intakeForward();
        }
        if(RobotContainer.operatorController.getRightBumperPressed()){
            intake.intakeReverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
