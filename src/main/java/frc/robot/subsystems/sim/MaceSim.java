package frc.robot.subsystems.sim;

import java.util.List;

import org.opencv.aruco.EstimateParameters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MacePoint;
import frc.robot.subsystems.MaceTrajectory;
import frc.robot.subsystems.MaceTrajectoryGenerator;

public class MaceSim extends SubsystemBase{

    private final ArmSim aSim;
    private final ElevatorSimulator eSim;

    
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_elevator = m_mech2d.getRoot("Elevator", 30, 30);

    private final MechanismLigament2d m_elevatorMech2d;
    private final MechanismLigament2d m_arm;


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
        SmartDashboard.putData("Arm Sim", m_mech2d);

    }

    @Override
    public void periodic() {
        aSim.run();
        eSim.run();

        
    }

    @Override
    public void simulationPeriodic() {
        aSim.simulate();
        eSim.simulate();
        m_arm.setAngle(aSim.getArmAngle() - 90);
        m_elevatorMech2d.setLength(eSim.getPosition());
    }

    
}
