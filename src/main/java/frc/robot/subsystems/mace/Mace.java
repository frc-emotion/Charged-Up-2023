

package frc.robot.subsystems.mace;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import org.opencv.aruco.EstimateParameters;
import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.SimConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.mace.MacePoint;
import frc.robot.subsystems.mace.MaceTrajectory;
import frc.robot.subsystems.mace.MaceTrajectoryGenerator;
import frc.robot.subsystems.mace.StoredTrajectories;

public class Mace extends SubsystemBase{

    //private final ArmSubsystem aSystem;
    //private final ElevatorSubsystem eSystem;
    private Timer trajTimer = new Timer();

    double times = 0;

    private MaceTrajectory currentTraj = null;

    private MaceTrajectory test, back;
    List<Double> values;

    private double armHoldingPosition;
    private double elevatorHoldingPostion;

    private boolean startTraj = false;
    private boolean backTraj = false;


   // public Mace(ArmSubsystem aSystem, ElevatorSubsystem eSystem){
     /*    this.aSystem = aSystem;
        this.eSystem = eSystem;
        //test = StoredTrajectories.HIGH_MACE_TRAJECTORY;
        //back = StoredTrajectories.BACK_HIGH_TRAJECTORY;

        SmartDashboard.putNumber("Height", eSystem.getHeight());
        SmartDashboard.putNumber("timer", trajTimer.get());
        //SmartDashboard.putNumber("totalTime", 0);
        trajTimer.reset();


        SmartDashboard.putNumber("ttR", test.totalTime());
        
    }

    public void stopAll(){
        aSystem.setArmSpeeds(0);
        eSystem.setElevatorVoltage(0);
    }

    private void runTrajectory(){

            trajTimer.start();
            
            //aSim.s(10);
           aSystem.setArmSpeeds(aSystem.getPIDOutputVolts(Units.rotationsToRadians(SimConstants.armMeterToRot(values.get(0)))));


            eSystem.setElevatorVoltage(/*eSim.getFeedForwardOutputVolts(values.get(4), values.get(5))*///eSystem.getPIDOutputVolts(values.get(3)));






    //}
    /**
     * if button presed:
     * startTrajectory = true;
     * current Traj = t;
     * 
     * if (startTrajectory = true){
     *      values = currentTraj.sampleMace();
     *      runTrajectory();
     * }
     * 
     */

    public void runMace(Supplier<Boolean> func, Supplier<Double> prep, MaceTrajectory front, MaceTrajectory backT) {
/* 
        if (prep.get() > 0.4){
            if (currentTraj == front && func.get() && !currentTraj.isActive(trajTimer.get())){
                startTraj = false;
                trajTimer.reset();
                currentTraj = backT;
                backTraj = true;

            } else if (func.get() && (currentTraj == null || currentTraj == backT)){
                backTraj = false;
                trajTimer.reset();
                currentTraj = front;
                startTraj = true;
            }


            if (backTraj){
                values = currentTraj.sampleMace(trajTimer.get());
                runTrajectory();
            }

            if (startTraj){
                values = currentTraj.sampleMace(trajTimer.get());
                runTrajectory();
            }

            if (currentTraj != null && (startTraj || backTraj)) {
                if (!currentTraj.isActive(trajTimer.get())){
                    trajTimer.stop();
                }
            }
    } else {
        stopAll();
        currentTraj = null;
        startTraj = false;
        backTraj = false;
    }
    } */
    }
}