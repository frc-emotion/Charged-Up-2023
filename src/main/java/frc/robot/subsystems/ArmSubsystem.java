package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;


public class ArmSubsystem extends SubsystemBase{

    private final CANSparkMax armMotor;
    //private final DutyCycleEncoder absoluteEncoder;
    private final RelativeEncoder armEncoder;
    private final ArmFeedforward armFeedForward;
    private final SparkMaxPIDController armController;
    private double pidVal, feedForwardVal, angularSpeed;
    

    
    public ArmSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                //resetPosition();
            } catch (Exception io) {
            }
        }).start();

        armMotor = new CANSparkMax(ArmConstants.armMotorPort, MotorType.kBrushless);
        
        armMotor.setSmartCurrentLimit(45);
        armMotor.setSecondaryCurrentLimit(45);
        armMotor.setIdleMode(IdleMode.kBrake);

        armEncoder = armMotor.getEncoder();
        
        //absoluteEncoder = new DutyCycleEncoder(ArmConstants.absoluteEncoderPort);
        armFeedForward = new ArmFeedforward(ArmConstants.armKS, ArmConstants.armKV, ArmConstants.armKG);
        armController = armMotor.getPIDController();
        armController.setP(SmartDashboard.getNumber("KP Constant", ArmConstants.armKP));
        armController.setD(SmartDashboard.getNumber("KD Constant", ArmConstants.armKD));
        armController.setI(SmartDashboard.getNumber("KI Constant", ArmConstants.armKI));


        //armController = new PIDController(SmartDashboard.getNumber("KP Constant", ArmConstants.armKP), SmartDashboard.getNumber("KD Constant", ArmConstants.armKD), SmartDashboard.getNumber("KI Constant", ArmConstants.armKI));
    }

    public void setArmAngle(){

        if(RobotContainer.operatorController.getRightStickButtonPressed()){
            feedForwardVal = armFeedForward.calculate(SmartDashboard.getNumber("Top Height", ArmConstants.TOP_HEIGHT), ArmConstants.MAX_ARM_VELOCITY);
            armController.setReference(SmartDashboard.getNumber("Top Height", ArmConstants.TOP_HEIGHT), ControlType.kPosition, 4, feedForwardVal);

            angularSpeed = armMotor.getAppliedOutput();
            
            MathUtil.clamp(angularSpeed, 0, 12);
            setArmSpeeds(angularSpeed);
        }
        else if(RobotContainer.operatorController.getLeftStickButtonPressed()){
            feedForwardVal = armFeedForward.calculate(SmartDashboard.getNumber("Middle Height", ArmConstants.MIDDLE_HEIGHT), ArmConstants.MAX_ARM_VELOCITY);
            armController.setReference(SmartDashboard.getNumber("Top Height", ArmConstants.MIDDLE_HEIGHT), ControlType.kPosition, 4, feedForwardVal);

            angularSpeed = armMotor.getAppliedOutput();
            
            MathUtil.clamp(angularSpeed, 0, 12); // Honestly considering the way the angularspeed is being used, might not even be needed
            setArmSpeeds(angularSpeed);
        }
        else if(RobotContainer.operatorController.getBackButtonPressed()){
            feedForwardVal = armFeedForward.calculate(SmartDashboard.getNumber("Low Height", ArmConstants.LOW_HEIGHT), ArmConstants.MAX_ARM_VELOCITY);
            armController.setReference(SmartDashboard.getNumber("Low Height", ArmConstants.LOW_HEIGHT), ControlType.kPosition, 4, feedForwardVal);

            angularSpeed = armMotor.getAppliedOutput();
            
            MathUtil.clamp(angularSpeed, 0, 12);
            setArmSpeeds(angularSpeed);
        }
    }
    
    public void setArmSpeeds(double armSpeeds){
        armMotor.set(armSpeeds);
    }

    public void stopArm(){
        armMotor.stopMotor();
    }

    //converts units to radians
    public void convertToMeters(){
        armEncoder.setPositionConversionFactor((2 * Math.PI)  / ArmConstants.ARM_GEAR_RATIO);
        armEncoder.setVelocityConversionFactor((2 * Math.PI) / (60 * ArmConstants.ARM_GEAR_RATIO));
    }

    public double getPosition(){
        return armEncoder.getPosition();
    }

    //public void resetPosition(){
        //armEncoder.reset();
    //}

    //public void getPositionOffset(){
      //  absoluteEncoder.getPositionOffset();
    //}

    @Override
    public void periodic(){
        //put values on smart dashboard
        SmartDashboard.putNumber("KP Constant", ArmConstants.armKP);
        SmartDashboard.putNumber("KD Constant", ArmConstants.armKD);
        SmartDashboard.putNumber("KI Constant", ArmConstants.armKI);

        SmartDashboard.putNumber("Arm speed", ArmConstants.ARM_SPEED);

        SmartDashboard.putNumber("Low Height", ArmConstants.LOW_HEIGHT);
        SmartDashboard.putNumber("Middle Height", ArmConstants.MIDDLE_HEIGHT);
        SmartDashboard.putNumber("Top Height", ArmConstants.TOP_HEIGHT);

        //get the height to the next setpoint periodically
        getPosition();
    }
}