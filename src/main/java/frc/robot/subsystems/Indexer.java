package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;;

public class Indexer extends SubsystemBase {

    private final CANSparkMax indexerMotorA;
    private final CANSparkMax indexerMotorB;

    double INDEXER_SPEED;

    public Indexer(){
        indexerMotorA = new CANSparkMax(IndexerConstants.INDEXER_A_MOTOR_PORT, MotorType.kBrushless);
        indexerMotorB = new CANSparkMax(IndexerConstants.INDEXER_B_MOTOR_PORT, MotorType.kBrushless);

        indexerMotorA.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorA.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorA.setIdleMode(IdleMode.kCoast);

        indexerMotorB.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorB.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        indexerMotorB.setIdleMode(IdleMode.kCoast);

        SmartDashboard.putNumber("Indexer Speed", IndexerConstants.INDEXER_SPEED);
    }

    public void indexerForward(){
        indexerMotorA.set(SmartDashboard.getNumber("Indexer Speed", IndexerConstants.INDEXER_SPEED));
        indexerMotorB.set(-SmartDashboard.getNumber("Indexer Speed", IndexerConstants.INDEXER_SPEED));
    }

    public void indexerReverse(){
        indexerMotorA.set(-SmartDashboard.getNumber("Indexer Speed", IndexerConstants.INDEXER_SPEED));
        indexerMotorB.set(SmartDashboard.getNumber("Indexer Speed", IndexerConstants.INDEXER_SPEED));
    }

    public void indexerStop(){
        indexerMotorA.set(0);
        indexerMotorB.set(0);
    }
    
    @Override
    public void periodic(){
        INDEXER_SPEED = SmartDashboard.getNumber("Indexer Speed", IndexerConstants.INDEXER_SPEED);
    }
}
    
