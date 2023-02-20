package frc.robot.subsystems.sim;

/**For abstraction purposes */
public interface Simulatable {
    
    public void onInit();
    public void run();
    public void simulate();
    public void disabledInit();
}
