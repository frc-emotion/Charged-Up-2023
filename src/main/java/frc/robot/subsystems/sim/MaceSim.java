package frc.robot.subsystems.sim;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.subsystems.MacePoint;
import frc.robot.subsystems.MaceTrajectory;
import frc.robot.subsystems.MaceTrajectoryGenerator;
import frc.robot.subsystems.StoredTrajectories;

public class MaceSim extends SubsystemBase{

    private final ArmSim aSim;
    private final ElevatorSimulator eSim;

    
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_elevator = m_mech2d.getRoot("Elevator", 30, 30);

    private final MechanismLigament2d m_elevatorMech2d;
    private final MechanismLigament2d m_arm;


    private Timer trajTimer = new Timer();



    double times = 0;

    private MaceTrajectory currentTraj = null;

    private MaceTrajectory t;


    public MaceSim(ArmSim aSim, ElevatorSimulator eSim){
        this.aSim = aSim;
        this.eSim = eSim;
 
        m_elevatorMech2d = m_elevator.append(
            new MechanismLigament2d(
                "Elevator", Units.metersToInches(eSim.getPosition()), 90));
        
        m_elevatorMech2d.setColor(new Color8Bit(Color.kBlue));

        m_arm = m_elevatorMech2d.append(
            new MechanismLigament2d(
                "Arm",
                30,
                aSim.getArmAngle(),
                6,
                new Color8Bit(Color.kYellow)));

        aSim.onInit();

        eSim.onInit();

        t = StoredTrajectories.TEST_TRAJECTORY;


        SmartDashboard.putData("Arm Sim", m_mech2d);
        SmartDashboard.putNumber("Angle", aSim.getArmAngle());
        SmartDashboard.putNumber("Height", Units.inchesToMeters(eSim.getPosition()));
        SmartDashboard.putNumber("timer", trajTimer.get());
        //SmartDashboard.putNumber("totalTime", 0);
        trajTimer.reset();

        SmartDashboard.putNumber("ttR", t.totalTime());
        
    }

    private void runTrajectory(){

        if (RobotContainer.simJoystick.getRawButton(3) && currentTraj == null){
            trajTimer.start();
            currentTraj = t;
            List<Double> values = currentTraj.sampleMace(trajTimer.get());

            //aSim.setArmVoltage(10);
         //   aSim.setArmVoltage(aSim.getFeedForwardOutputVolts(Units.rotationsToRadians(SimConstants.armMeterToRot(values.get(0))), Units.rotationsPerMinuteToRadiansPerSecond(SimConstants.armMPStoRPM(values.get(1))),
         //       SimConstants.armMPSStoRadSS(values.get(2))) + aSim.getPIDOutputVolts(Units.rotationsToRadians(SimConstants.armMeterToRot(values.get(0)))));


            eSim.setElevatorVoltage(eSim.getFeedForwardOutputVolts(values.get(4), values.get(5)) + eSim.getPIDOutputVolts(values.get(3)));
        }

        if (currentTraj != null) {
            if (!currentTraj.isActive(trajTimer.get())){
                currentTraj = null;
                trajTimer.reset();
            }
        }



    }

    @Override
    public void periodic() {
        runTrajectory();

        SmartDashboard.putNumber("timer", trajTimer.get());
        SmartDashboard.putNumber("totalTime", t.getTotalTimeSeconds());
        SmartDashboard.putNumber("Angle", aSim.getArmAngle());
        SmartDashboard.putNumber("Height", Units.inchesToMeters(eSim.getPosition()));
    }

    @Override
    public void simulationPeriodic() {
        aSim.simulate();
        eSim.simulate();
        m_arm.setAngle(aSim.getArmAngle() - 90);
        m_elevatorMech2d.setLength(eSim.getPosition());

        SmartDashboard.putNumber("Angle", aSim.getArmAngle());
        SmartDashboard.putNumber("Height", Units.inchesToMeters(eSim.getPosition()));
    }


    public MaceTrajectory generateTrajectory() {

        // 2018 cross scale auto waypoints.
        var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
            Rotation2d.fromDegrees(-180));
        var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
            Rotation2d.fromDegrees(-160));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));
    
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(true);
    
        var trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart,
            interiorWaypoints,
            crossScale,
            config);

        return new MaceTrajectory(trajectory);
      }

    
}
