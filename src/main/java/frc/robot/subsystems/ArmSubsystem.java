package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    private final DutyCycleEncoder absoluteEncoder;
    private final ArmFeedforward armFeedForward;
    private final PIDController armController;
    private double pidVal, feedForwardVal;
    

    
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
        armController = new PIDController(ArmConstants.armKP, ArmConstants.armKD, ArmConstants.armKI);
    }

    public void setArmAngle(){

        if(RobotContainer.operatorController.getRightStickButtonPressed()){
            armController.setSetpoint(ArmConstants.TOP_HEIGHT);
            pidVal = armController.calculate(getAbsolutePosition());
            feedForwardVal = armFeedForward.calculate(ArmConstants.TOP_HEIGHT, ArmConstants.MAX_ARM_VELOCITY);

            MathUtil.clamp(pidVal, 0, 12);
            setArmSpeeds(feedForwardVal + pidVal);
        }
        else if(RobotContainer.operatorController.getRightBumperPressed()){
            armController.setSetpoint(ArmConstants.MIDDLE_HEIGHT);
            pidVal = armController.calculate(getAbsolutePosition());
            feedForwardVal = armFeedForward.calculate(ArmConstants.MIDDLE_HEIGHT, ArmConstants.MAX_ARM_VELOCITY);

            MathUtil.clamp(pidVal, 0, 12);
            setArmSpeeds(feedForwardVal + pidVal);
        }
        else if(RobotContainer.operatorController.getLeftBumperPressed()){
            armController.setSetpoint(ArmConstants.LOW_HEIGHT);
            pidVal = armController.calculate(getAbsolutePosition());
            feedForwardVal = armFeedForward.calculate(ArmConstants.LOW_HEIGHT, ArmConstants.MAX_ARM_VELOCITY);

            MathUtil.clamp(pidVal, 0, 12);
            setArmSpeeds(feedForwardVal + pidVal);
        }
    }
    
    public void setArmSpeeds(double armSpeeds){
        armMotor.set(armSpeeds);
    }

    public void stopArm(){
        armMotor.stopMotor();
    }

    //converts inches to meters
    public void convertToMeters(){
        armMotor.getEncoder().setPositionConversionFactor(0.001 * ArmConstants.ARM_GEAR_RATIO);
        armMotor.getEncoder().setVelocityConversionFactor((Math.PI / 30) * ArmConstants.ARM_SPROCKET_RADIUS);
    }

    public double getAbsolutePosition(){
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetPosition(){
        absoluteEncoder.reset();
    }

    public void getPositionOffset(){
        absoluteEncoder.getPositionOffset();
    }

    @Override
    public void periodic(){
        //put values on smart dashboard
        SmartDashboard.putNumber("KP Constant", ArmConstants.armKP);
        SmartDashboard.putNumber("KD Constant", ArmConstants.armKD);
        SmartDashboard.putNumber("KI Constant", ArmConstants.armKI);

        //get the height to the next setpoint periodically
        getAbsolutePosition();
    }
}