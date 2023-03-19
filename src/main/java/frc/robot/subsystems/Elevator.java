package frc.robot.subsystems;
import java.lang.Math;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.playingwithfusion.TimeOfFlight;



public class Elevator extends SubsystemBase{
 /*    private ElevatorFeedforward feedForward;
    private PIDController elevatorController;
    private CANSparkMax elevatorMotor;
    private double low, middle, high;
    private TimeOfFlight positionSensor;
    double setpoint, pidValue, feedForwardVal;

    public Elevator(){
        feedForward = new ElevatorFeedforward(Constants.ElevatorConstants.KS, Constants.ElevatorConstants.KG, Constants.ElevatorConstants.KV);
        elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.ELEVATORMOTOR_ID, MotorType.kBrushless);

        elevatorController = new PIDController(SmartDashboard.getNumber("KP Value", Constants.ElevatorConstants.KP), 
        SmartDashboard.getNumber("KD Value", 0), 
        SmartDashboard.getNumber("KI Value", Constants.ElevatorConstants.KI));

        positionSensor = new TimeOfFlight(Constants.ElevatorConstants.CANID);

        resetHeight();
        convertToMeters();

        low = Constants.ElevatorConstants.LOWLEVEL;
        middle = Constants.ElevatorConstants.MIDDLELEVEL;
         high = Constants.ElevatorConstants.HIGHLEVEL;
    }

    //the xbox controller buttons are temporary
    //PID for each level on the elevator to ease into position
    public void setHeight(){

        if(RobotContainer.operatorController.getAButton()){
            
            elevatorController.setSetpoint(high);
            pidValue = elevatorController.calculate(getHeight());
            feedForwardVal = feedForward.calculate(0.02);

            //MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(pidValue);
        }
        else if(RobotContainer.operatorController.getBButton()){

            elevatorController.setSetpoint(middle);
            pidValue = elevatorController.calculate(getHeight());
            feedForwardVal = feedForward.calculate(0.02);

            //MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(pidValue);
        }
        else if(RobotContainer.operatorController.getXButton()){

            elevatorController.setSetpoint(low);
            pidValue = elevatorController.calculate(getHeight());
            feedForwardVal = feedForward.calculate(0.02);

            //MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(pidValue);
        }
        else {
            stopElevator();
        }
    }
    //allows for manual control as backup
    public void manualMove(Supplier<Double> func){
        if(getHeight() < Constants.ElevatorConstants.MAXLEVEL && getHeight() > Constants.ElevatorConstants.MINLEVEL ){
            if(func.get() > OIConstants.kDeadband){
                elevatorMotor.set(Constants.ElevatorConstants.ELEVATORMOTORSPEED);
            }  else if(func.get() < -OIConstants.kDeadband){
                elevatorMotor.set(-Constants.ElevatorConstants.ELEVATORMOTORSPEED);
            }else{
                stopElevator();
            }
        }
        
    }
    //return the height the position control calculates
    public double getHeight(){
        return positionSensor.getRange() * 0.001;
    }

    
    //converts inches to meters
    public void convertToMeters(){
        elevatorMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * Constants.ElevatorConstants.SPROCKETRADIUS * (1/8));
        elevatorMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI * Constants.ElevatorConstants.SPROCKETRADIUS * (1/8) / 60);
    }

    public void stopElevator(){
        elevatorMotor.stopMotor();
    }

    public double getError(){
        return elevatorController.getPositionError();
    }
    public void resetHeight(){
        elevatorMotor.getEncoder().setPosition(getHeight());
    }
    @Override
    public void periodic(){
        //put values on smart dashboard
        SmartDashboard.putNumber("KP Constant", Constants.ElevatorConstants.KP);
        SmartDashboard.putNumber("KD Constant", Constants.ElevatorConstants.KD);
        SmartDashboard.putNumber("KI Constant", Constants.ElevatorConstants.KI);
        
        SmartDashboard.putNumber("Range from sensor", getHeight());
        SmartDashboard.putNumber("Position", elevatorMotor.getEncoder().getPosition());

        //get the height to the next setpoint periodically
    }
    */
}   

