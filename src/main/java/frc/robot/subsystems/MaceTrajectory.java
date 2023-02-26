package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;

public class MaceTrajectory extends Trajectory {

    private Trajectory mTrajectory;

    public MaceTrajectory(Trajectory m_traj) {
        mTrajectory = m_traj;
    }

    public List<Double> sampleMace(double time) {

        var dt = totalTime() / (allStates().size() - 1);

        int prevIndex = (int) Math.floor(time / dt);
        int nextIndex = (int) Math.ceil(time / dt);
        if (nextIndex == prevIndex)
            nextIndex++;
        int secondPrevIndex = prevIndex - 1;
        int secondNextIndex = nextIndex + 1;

        prevIndex = MathUtil.clamp(prevIndex, 0, allStates().size() - 1);
        nextIndex = MathUtil.clamp(nextIndex, 0, allStates().size() - 1);
        secondPrevIndex = MathUtil.clamp(secondPrevIndex, 0, allStates().size() - 1);
        secondNextIndex = MathUtil.clamp(secondNextIndex, 0, allStates().size() - 1);

        // Calculate positions in 2 ways
        double aPosition_0 = MathUtil.interpolate(
                new MacePoint(allStates().get(prevIndex).poseMeters).getArmAngle(),
                new MacePoint(allStates().get(nextIndex).poseMeters).getArmAngle(), (time % dt) / dt);
        double ePosition_0 = MathUtil.interpolate(
                new MacePoint(allStates().get(prevIndex).poseMeters).getElevatorHeight(),
                new MacePoint(allStates().get(nextIndex).poseMeters).getElevatorHeight(), (time % dt) / dt);

        // #2
        double aPosition_1 = new MacePoint(mTrajectory.sample(time).poseMeters).getArmAngle();
        double ePosition_1 = new MacePoint(mTrajectory.sample(time).poseMeters).getElevatorHeight();

        // calculate velocities
        double aVelocity = (new MacePoint(allStates().get(nextIndex).poseMeters).getArmAngle()
                - new MacePoint(allStates().get(prevIndex).poseMeters).getArmAngle()) / dt;
        double eVelocity = (new MacePoint(allStates().get(nextIndex).poseMeters).getElevatorHeight()
                - new MacePoint(allStates().get(prevIndex).poseMeters).getElevatorHeight()) / dt;

        // calculate accelerations
        double acceleration_A, acceleration_E;
        if ((time % dt) / dt < 0.5) {
            double aPrevVelocity = (new MacePoint(allStates().get(prevIndex).poseMeters).getArmAngle()
                    - new MacePoint(allStates().get(secondPrevIndex).poseMeters).getArmAngle()) / dt;
            double ePrevVelocity = (new MacePoint(allStates().get(prevIndex).poseMeters).getElevatorHeight()
                    - new MacePoint(allStates().get(secondPrevIndex).poseMeters).getElevatorHeight()) / dt;
            acceleration_A = (aVelocity - aPrevVelocity) / dt;
            acceleration_E = (eVelocity - ePrevVelocity) / dt;
        } else {
            double aNextVelocity = (new MacePoint(allStates().get(secondNextIndex).poseMeters).getArmAngle()
                    - new MacePoint(allStates().get(nextIndex).poseMeters).getArmAngle()) / dt;
            double eNextVelocity = (new MacePoint(allStates().get(secondNextIndex).poseMeters).getElevatorHeight()
                    - new MacePoint(allStates().get(nextIndex).poseMeters).getElevatorHeight()) / dt;
            acceleration_A = (aNextVelocity - aVelocity) / dt;
            acceleration_E = (eNextVelocity - eVelocity) / dt;
        }

        return new ArrayList<Double>() {
            {
                add(aPosition_0); //meters
                add(aVelocity); //mps
                add(acceleration_A); //mps^2
                add(ePosition_0); //m
                add(eVelocity);// mps
                add(acceleration_E); //mps^2
            }
        };

    }

    public double totalTime() {
        return mTrajectory.getTotalTimeSeconds();
    }

    public List<State> allStates() {
        return mTrajectory.getStates();
    }
}
