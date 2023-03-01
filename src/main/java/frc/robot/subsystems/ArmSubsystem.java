package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;


public class ArmSubsystem extends SubsystemBase{

    private final CANSparkMax armMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final ArmFeedforward armFeedForward;
    private final TrapezoidProfile.Constraints armConstraints;
    private final SparkMaxPIDController sparkArmController; // Not entirely sure which one we want to be using right now so will use them based on the ways I'm seeing other people use them
    private final ProfiledPIDController profiledArmController;
    

    
    public ArmSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetPosition();
            } catch (Exception io) {
            }
        }).start();

        armMotor = new CANSparkMax(ArmConstants.armMotorPort, MotorType.kBrushless);

        armMotor.setSmartCurrentLimit(45);
        armMotor.setSecondaryCurrentLimit(45);
        armMotor.setIdleMode(IdleMode.kBrake);

        absoluteEncoder = new DutyCycleEncoder(ArmConstants.absoluteEncoderPort);
        armFeedForward = new ArmFeedforward(ArmConstants.armKS, ArmConstants.armKV, ArmConstants.armKG);
        
        armConstraints = new TrapezoidProfile.Constraints(ArmConstants.MAX_ARM_VELOCITY, ArmConstants.MAX_ARM_ACCELERATION); // change this
        profiledArmController = new ProfiledPIDController(ArmConstants.armKP, ArmConstants.armKD, ArmConstants.armKI, armConstraints); // change this
        sparkArmController = armMotor.getPIDController();

    }
    
    //public void setArmHeightP(){ // Utilizes ProfiledPIDController
        //double PIDValue = profiledArmController.calculate(getAbsolutePosition());
        //double FFValue = armFeedForward.calculate(getAbsolutePosition(), ArmConstants.MAX_ARM_VELOCITY); // Don't think it will work if the Absolute Encoder's original position is assumed to be perpedicular to the floor
    //}

    public void setArmHeightS(){ // Utilizes SparkMaxPIDController
        if (RobotContainer.operatorController.getAButtonPressed()){
            sparkArmController.setReference(ArmConstants.TOP_HEIGHT, ControlType.kPosition, 5, armFeedForward.calculate(getAbsolutePosition(), ArmConstants.ARM_SPEED));
        }
        if (RobotContainer.operatorController.getBButtonPressed()){
            sparkArmController.setReference(ArmConstants.MIDDLE_HEIGHT, ControlType.kPosition, 5, armFeedForward.calculate(getAbsolutePosition(), ArmConstants.ARM_SPEED));
        }
        if (RobotContainer.operatorController.getXButtonPressed()){
            sparkArmController.setReference(ArmConstants.LOW_HEIGHT, ControlType.kPosition, 5, armFeedForward.calculate(getAbsolutePosition(), ArmConstants.ARM_SPEED));
        }
    }
    
    public void setArmSpeeds(double armSpeeds){
        armMotor.set(armSpeeds);
    }

    public void stopArm(){
        armMotor.stopMotor();
    }

    public double getAbsolutePosition(){
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetPosition(){
        absoluteEncoder.reset();
    }
}