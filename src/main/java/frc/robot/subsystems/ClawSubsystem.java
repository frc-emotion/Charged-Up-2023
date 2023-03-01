package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
    
public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax claw = new CANSparkMax(ClawConstants.CLAW, MotorType.kBrushless);

    public ClawSubsystem() {
     claw.setIdleMode(IdleMode.kBrake);
     claw.setInverted(false); //check 
     //claw.enableSoftLimit(null, false); //check
    }

    public void setClawMotor(double speed) {
        claw.set(speed);
    } 

    public double getClawCurrent() {
        return claw.getOutputCurrent();
    }

    public void stopClaw() {
        claw.stopMotor();
    }
    

}




