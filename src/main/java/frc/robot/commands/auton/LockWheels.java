package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsytem;

public class LockWheels extends CommandBase {
    
        private final SwerveSubsytem swerve;
    
        public LockWheels(SwerveSubsytem swerve) {
            this.swerve = swerve;
            addRequirements(swerve);
        }
    
        @Override
        public void initialize() {

        }
    
        @Override
        public void execute() {
            Rotation2d leftRotation = new Rotation2d(Math.PI/4);
            Rotation2d rightRotation = new Rotation2d(-Math.PI/4);

    
            SwerveModuleState[] stoppedStates = {new SwerveModuleState(0, rightRotation), 
                                                 new SwerveModuleState(0, leftRotation),
                                                 new SwerveModuleState(0, rightRotation),
                                                 new SwerveModuleState(0, leftRotation)                                           
                                                };
            swerve.setModuleStates(stoppedStates, true);

        }   
    
        @Override
        public void end(boolean interrupted) {
            swerve.stopModules();
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    
}
