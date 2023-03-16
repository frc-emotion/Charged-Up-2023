package frc.robot.subsystems;
import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.playingwithfusion.TimeOfFlight;



public class Elevator extends SubsystemBase{
    private ElevatorFeedforward feedForward;
    private PIDController elevatorController;
    private CANSparkMax elevatorMotor;
    private double low, middle, high;
    private TimeOfFlight positionSensor;
    double setpoint, velocity, pidValue, feedForwardVal;

    public Elevator(){
        feedForward = new ElevatorFeedforward(5, 5, 1);
        elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
        
        elevatorController = new PIDController(SmartDashboard.getNumber("KP Value", Constants.ElevatorConstants.KP), 
        SmartDashboard.getNumber("KD Value", Constants.ElevatorConstants.KD), 
        SmartDashboard.getNumber("KI Value", Constants.ElevatorConstants.KI));

        positionSensor = new TimeOfFlight(Constants.ElevatorConstants.CANID);

        low = SmartDashboard.getNumber("Low Level", Constants.ElevatorConstants.LOWLEVEL);
        middle = SmartDashboard.getNumber("Middle Level", Constants.ElevatorConstants.MIDDLELEVEL);
        high = SmartDashboard.getNumber("High Level", Constants.ElevatorConstants.HIGHLEVEL);

        

        elevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT);
        elevatorMotor.setSecondaryCurrentLimit(Constants.ElevatorConstants.SECOND_CURRENT_LIMIT);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
    }

    //the xbox controller buttons are temporary
    //PID for each level on the elevator to ease into position
    public void setHeight(){

        if(RobotContainer.operatorController.getAButtonPressed()){
            elevatorController.setSetpoint(high);
            pidValue = elevatorController.calculate(getHeight());
            feedForwardVal = feedForward.calculate(positionSensor.getRange());

            MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(feedForwardVal + pidValue);
        }
        else if(RobotContainer.operatorController.getBButtonPressed()){
            elevatorController.setSetpoint(middle);
            pidValue = elevatorController.calculate(getHeight());
            feedForwardVal = feedForward.calculate(positionSensor.getRange());

            MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(feedForwardVal + pidValue);
        }
        else if(RobotContainer.operatorController.getXButtonPressed()){
            elevatorController.setSetpoint(low);
            pidValue = elevatorController.calculate(getHeight());
            feedForwardVal = feedForward.calculate(positionSensor.getRange());

            MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(feedForwardVal + pidValue);
        }
    }
    //allows for manual control as backup
    public void manualMove(){
        if(positionSensor.getRange() < Constants.ElevatorConstants.MAXLEVEL * 1000 && positionSensor.getRange() > Constants.ElevatorConstants.MINLEVEL * 1000){
            if(RobotContainer.operatorController.getLeftY() > Constants.ElevatorConstants.MINTHRESHOLD){
                elevatorMotor.set(Constants.ElevatorConstants.ELEVATORMOTORSPEED);
            }   else if(RobotContainer.operatorController.getLeftY() < -Constants.ElevatorConstants.MINTHRESHOLD){
                elevatorMotor.set(-Constants.ElevatorConstants.ELEVATORMOTORSPEED);
            }
        }
        
    }
    //return the height the position control calculates
    public double getHeight(){
        return positionSensor.getRange();
    }

    
    //converts inches to meters
    //public void convertToMeters(){
        //elevatorMotor.getEncoder().setPositionConversionFactor(0.001 * Constants.ElevatorConstants.GEARRATIO);
        //elevatorMotor.getEncoder().setVelocityConversionFactor((Math.PI / 30) * Constants.ElevatorConstants.SPROCKETRADIUS);
    //}

    public void stopElevator(){
        elevatorMotor.stopMotor();
    }

    public double getError(){
        return elevatorController.getPositionError();
    }

    public void periodic(){
        //put values on smart dashboard
        SmartDashboard.putNumber("KP Constant", Constants.ElevatorConstants.KP);
        SmartDashboard.putNumber("KD Constant", Constants.ElevatorConstants.KD);
        SmartDashboard.putNumber("KI Constant", Constants.ElevatorConstants.KI);

        SmartDashboard.putNumber("Low Level", Constants.ElevatorConstants.LOWLEVEL);
        SmartDashboard.putNumber("Middle Level", Constants.ElevatorConstants.MIDDLELEVEL);
        SmartDashboard.putNumber("High Level", Constants.ElevatorConstants.HIGHLEVEL);



        

        



        //get the height to the next setpoint periodically
        getHeight();

    }
    
}   

