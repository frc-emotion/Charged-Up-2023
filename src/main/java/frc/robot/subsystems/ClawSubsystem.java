package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants.MotorPorts;
    
public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax claw = new CANSparkMax(Constants.MotorPorts.CLAW, MotorType.kBrushless);

    public ClawSubsystem() {
     claw.setIdleMode(IdleMode.kBrake);
     claw.setInverted(false); //check 
     claw.enableSoftLimit(null, false); //check

   /*  SparkMaxPIDController clawController = claw.getPIDController();
     clawController.setP(0);
     clawController.setI(0); */
    }

    public void setClawMotor(double speed) {
        claw.set(speed);
    } 

    public double getClawCurrent() {
        return claw.getOutputCurrent();
    }

}




