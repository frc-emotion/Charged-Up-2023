package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends CommandBase {
    
    private Indexer indexer;

    public RunIndexer(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }


    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        if(RobotContainer.operatorController.getLeftBumperPressed()){
            indexer.indexerForward();
        }
        if(RobotContainer.operatorController.getRightBumperPressed()){
            indexer.indexerForward();
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
