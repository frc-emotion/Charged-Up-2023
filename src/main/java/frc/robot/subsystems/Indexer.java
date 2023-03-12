package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;;

public class Indexer extends SubsystemBase {

    private final CANSparkMax indexerMotorA;
    private final CANSparkMax indexerMotorB;

    public Indexer(){
        indexerMotorA = new CANSparkMax(IndexerConstants.INDEXER_A_MOTOR_PORT, MotorType.kBrushless);
        indexerMotorB = new CANSparkMax(IndexerConstants.INDEXER_B_MOTOR_PORT, MotorType.kBrushless);

        indexerMotorA.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorA.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorA.setIdleMode(IdleMode.kBrake);

        indexerMotorB.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorB.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorB.setIdleMode(IdleMode.kBrake);
    }

    public void indexerForward(){
        indexerMotorA.set(IntakeConstants.INTAKE_SPEED);
        indexerMotorB.set(-IntakeConstants.INTAKE_SPEED);
    }

    public void intakeReverse(){
        indexerMotorA.set(-IntakeConstants.INTAKE_SPEED);
        indexerMotorB.set(IntakeConstants.INTAKE_SPEED);
    }

    public void indexerStop(){
        indexerMotorA.set(0);
        indexerMotorB.set(0);

    }
    
}
    
