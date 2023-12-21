package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;




public class ArmSubsystem extends SubsystemBase{


    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final ArmFeedforward armFeedForward;
    private final ProfiledPIDController armController;
    private double pidVal; // feedForwardVal, angularSpeed <-- Removed
   
    public ArmSubsystem(){


        armMotor = new CANSparkMax(ArmConstants.armMotorPort, MotorType.kBrushless);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
       
        armMotor.setSmartCurrentLimit(45);
        armMotor.setSecondaryCurrentLimit(45);
        armMotor.setIdleMode(IdleMode.kBrake);


        armEncoder = armMotor.getEncoder();
       
       
        armFeedForward = new ArmFeedforward(ArmConstants.armKS, ArmConstants.armKG, ArmConstants.armKV);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.8, 1.0);

        armController = new ProfiledPIDController(ArmConstants.armKP , ArmConstants.armKI, ArmConstants.armKD,
        constraints);


        convertToMeters();
        resetPosition();

        SmartDashboard.putNumber("KP Constant", ArmConstants.armKP);
        SmartDashboard.putNumber("KD Constant", ArmConstants.armKD);
        SmartDashboard.putNumber("KI Constant", ArmConstants.armKI);


       
        SmartDashboard.putNumber("KS Constant", ArmConstants.armKS);
        SmartDashboard.putNumber("KV Constant", ArmConstants.armKV);
        SmartDashboard.putNumber("KG Constant", ArmConstants.armKG);


        SmartDashboard.putNumber("Arm speed", ArmConstants.ARM_SPEED);


        SmartDashboard.putNumber("Low Height", ArmConstants.LOW_HEIGHT);
        SmartDashboard.putNumber("Middle Height", ArmConstants.MIDDLE_HEIGHT);
        SmartDashboard.putNumber("Top Height", ArmConstants.TOP_HEIGHT);


    }

    public double getArmAngle(){
        return Units.radiansToDegrees(getPosition());
    }

    public double getPIDOutputVolts(double setpoint){
        return armController.calculate(getPosition(), setpoint);
    }

    public double getFeedForwardOutputVolts(double d, double v, double a){
        return armFeedForward.calculate(d, v, a);

    }

    public void setArmAuto(double setpoint){
        armController.setGoal(Units.degreesToRadians(setpoint));
        pidVal = armController.calculate(getPosition());
        setArmSpeeds(pidVal);
    }
    
    public void setArmAngle(){
        if(RobotContainer.operatorController.getBButton()){
            armController.setGoal(Units.degreesToRadians(-30));
            pidVal = armController.calculate(getPosition());

            setArmSpeeds(pidVal);
        }

        else{
            stopArm();
        }
    }
   
    public void setArmSpeeds(double armSpeeds){
        armMotor.set(armSpeeds);
    }


    public void stopArm(){
        armMotor.stopMotor();
    }


    public double getArmSpeeds(){
        return armMotor.get();
    }


    //converts units to radians
    public void convertToMeters(){
        armEncoder.setPositionConversionFactor(((2 * Math.PI) / ArmConstants.ARM_GEAR_RATIO));
        armEncoder.setVelocityConversionFactor((2 * Math.PI) / (60 * ArmConstants.ARM_GEAR_RATIO));
    }


    public double getPosition(){
        return armEncoder.getPosition();
    }


    public void resetPosition(){
        armEncoder.setPosition(Units.degreesToRadians(0));
        //armEncoder.setPosition(Math.PI/2);
    }


    //public void getPositionOffset(){
      //  absoluteEncoder.getPositionOffset();
    //}


    @Override
    public void periodic(){
        //put values on smart dashboard
        //armController.setP(SmartDashboard.getNumber("KP Constant", ArmConstants.armKP));
        //armController.setD(SmartDashboard.getNumber("KD Constant", ArmConstants.armKD));
        //armController.setI(SmartDashboard.getNumber("KI Constant", ArmConstants.armKI));


        //get the height to the next setpoint periodically
        SmartDashboard.putNumber("Arm Pose", Units.radiansToDegrees(getPosition()));


        //System.out.println("Arm Speeds " + getArmSpeeds());
    }
}
