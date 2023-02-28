package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;;

public class Indexer extends SubsystemBase {

    private final CANSparkMax indexerMotor;

    public Indexer(){
        indexerMotor = new CANSparkMax(IndexerConstants.INDEXER_MOTOR_PORT, MotorType.kBrushless);

        indexerMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotor.setIdleMode(IdleMode.kBrake);
    }

    public void indexerForward(){
        indexerMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void intakeReverse(){
        indexerMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    public void indexerStop(){
        indexerMotor.set(0);
    }
    
}
    
