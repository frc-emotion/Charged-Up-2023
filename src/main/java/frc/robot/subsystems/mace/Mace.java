

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

    private final ArmSubsystem aSystem;
    private final ElevatorSubsystem eSystem;
    private Timer trajTimer = new Timer();

    double times = 0;

    private MaceTrajectory currentTraj = null;

    private MaceTrajectory test;
    List<Double> values;

    private double armHoldingPosition;
    private double elevatorHoldingPostion;

    private boolean startTraj = false;


    public Mace(ArmSubsystem aSystem, ElevatorSubsystem eSystem){
        this.aSystem = aSystem;
        this.eSystem = eSystem;
        test = StoredTrajectories.TEST_TRAJECTORY;

        SmartDashboard.putNumber("Angle", aSystem.getArmAngle());
        SmartDashboard.putNumber("Height", eSystem.getHeight());
        SmartDashboard.putNumber("timer", trajTimer.get());
        //SmartDashboard.putNumber("totalTime", 0);
        trajTimer.reset();

        //elevatorHoldingPostion = eSim.getPIDOutputVolts(t.sampleMace(t.totalTime()).get(3));
        //armHoldingPosition = aSystem.getPIDOutputVolts(t.sampleMace(t.totalTime()).get(0));

        SmartDashboard.putNumber("ttR", test.totalTime());
        
    }

    private void runTrajectory(){

            trajTimer.start();
            
            //aSim.s(10);
            aSystem.setArmSpeeds(aSystem.getFeedForwardOutputVolts(Units.rotationsToRadians(SimConstants.armMeterToRot(values.get(0))), Units.rotationsPerMinuteToRadiansPerSecond(SimConstants.armMPStoRPM(values.get(1))),
                SimConstants.armMPSStoRadSS(values.get(2)))+  aSystem.getPIDOutputVolts(Units.rotationsToRadians(SimConstants.armMeterToRot(values.get(0)))));


            eSystem.setElevatorVoltage(/*eSim.getFeedForwardOutputVolts(values.get(4), values.get(5))*/eSystem.getPIDOutputVolts(values.get(3)));
    }





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

    public void runMace(Supplier<Boolean> func) {
        if (func.get()){
            //trajTimer.reset();
            currentTraj = test;
            startTraj = true;
        }
        /*if (func.get() && currentTraj !=null){
            startTraj = false;
            currentTraj = null;
        } */

        if (startTraj){
            values = currentTraj.sampleMace(trajTimer.get());
            runTrajectory();
        }

        if (currentTraj != null && startTraj) {
            if (!currentTraj.isActive(trajTimer.get())){
                trajTimer.stop();
                startTraj = false;
            }
        }

        if (values != null){
            SmartDashboard.putNumber("elePose", Units.metersToInches(values.get(3)));
            SmartDashboard.putNumber("armPose", Units.radiansToDegrees(Units.rotationsToRadians(SimConstants.armMeterToRot(values.get(0)))));
        }
        SmartDashboard.putNumber("timer", trajTimer.get());
    }
    
}