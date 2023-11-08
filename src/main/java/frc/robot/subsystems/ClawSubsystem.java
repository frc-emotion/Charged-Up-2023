package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
    
public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax claw;
    private int gamePieceType;

    public ClawSubsystem() {
    
     claw = new CANSparkMax(ClawConstants.CLAW, MotorType.kBrushless);
     
     claw.setSmartCurrentLimit(ClawConstants.CURRENT_LIMIT);
     claw.setSecondaryCurrentLimit(ClawConstants.SECOND_CURRENT_LIMIT);
     claw.setIdleMode(IdleMode.kBrake);
     claw.setInverted(false); 

     gamePieceType = 1;
    }

    public int getPieceType() {
        return gamePieceType;
    }

    public void switchGamePieceType() {
        if (gamePieceType == 0) {
            gamePieceType = 1;
        } else if (gamePieceType == 1) {
            gamePieceType = 0;
        }
    }

    public void setClawMotor(double speed) {
        claw.set(speed);
    } 

    public double getClawCurrent() {
        return claw.getOutputCurrent();
    }

    public double getSpeed(){
        return claw.get();
    }
    
    public double signedOutputCurrent(){
        return claw.get() < 0 ? getClawCurrent() * -1 : getClawCurrent();
    }

    public void stopClaw() {
        claw.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Output Current", getClawCurrent());
    }
    

}
