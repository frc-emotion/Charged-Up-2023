package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.dashboard.TabManager;
import frc.robot.util.dashboard.TabManager.SubsystemTab;
import frc.robot.util.vision.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Pair;
import java.util.Optional;

/**
 * Main Swerve Subsytem class
 * Holds gyro and odometry methods
 */
public class SwerveSubsytem extends SubsystemBase {

    private final SwerveModuleNeoFalcon frontLeft = new SwerveModuleNeoFalcon(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModuleNeoFalcon frontRight = new SwerveModuleNeoFalcon(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModuleNeoFalcon backLeft = new SwerveModuleNeoFalcon(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModuleNeoFalcon backRight = new SwerveModuleNeoFalcon(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public PoseEstimator visionPoseEstimator;

    private final SwerveModulePosition[] modulePositions =  new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, 
        new Rotation2d(0), // Rotation and Pose are 0 because auto sets them to a different value anyway. 
        modulePositions, 
        new Pose2d()); 

    private ChassisSpeeds robotSpeeds;

    private ShuffleboardLayout frontLeftData;
    private ShuffleboardLayout frontRightData;
    private ShuffleboardLayout backLeftData;
    private ShuffleboardLayout backRightData;
    private Field2d m_field;

    public SwerveSubsytem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception io) {
            }
        }).start();

        initShuffleboard();

    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getCurrentPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public double getPitch(){
        return Units.degreesToRadians((gyro.getPitch()));
    }

    //Resets current pose to a specified pose. 
    public void resetOdometry(Pose2d pose){
        poseEstimator.resetPosition(
            getRotation2d(), 
            getModulePositions(),
            pose);
    }

    public ChassisSpeeds getChassisSpeeds(){
        return robotSpeeds;
    }
    
    public void setChassisSpeeds(ChassisSpeeds speeds){
        robotSpeeds = speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }    
    

    @Override
    public void periodic() {

        //Updates with drivetrain sensors.
        poseEstimator.update(                   
            getRotation2d(), 
            getModulePositions());

        //Pair<Pose2d, Double> result = visionPoseEstimator.getEstimatedPose();  

        //Adds vision 
        //poseEstimator.addVisionMeasurement(result.getFirst(), result.getSecond()); 

        m_field.setRobotPose(getCurrentPose());
        System.out.println(gyro.getPitch());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    private void initShuffleboard(){
        ShuffleboardTab moduleData = TabManager.getInstance().accessTab(SubsystemTab.DRIVETRAIN);
        frontLeftData = moduleData.getLayout("Front Left", BuiltInLayouts.kList);
        frontRightData = moduleData.getLayout("Front Right", BuiltInLayouts.kList);
        backLeftData = moduleData.getLayout("Back Left", BuiltInLayouts.kList);
        backRightData = moduleData.getLayout("Back Right", BuiltInLayouts.kList);
        fillList(frontLeft, frontLeftData);
        fillList(frontRight, frontRightData);
        fillList(backLeft, backLeftData);
        fillList(backRight, backRightData);

        m_field = new Field2d();

        TabManager.getInstance().addFieldWidget(TabManager.getInstance().accessTab(SubsystemTab.AUTON), BuiltInWidgets.kField, "Pose", m_field,
        new int[] { 0, 0 }, new int[] { 6, 4 });
       // KPLevel = TabManager.getInstance().addFieldWidget(TabManager.getInstance().accessTab(SubsystemTab.AUTON), BuiltInWidgets.kField, "Pose", m_field,
        //new int[] { 0, 0 }, new int[] { 6, 4 });
    }

    private void fillList(SwerveModuleNeoFalcon module, ShuffleboardLayout layout){
        layout.addNumber("Absolute Position", () -> module.getAbsolutePostion());
        layout.addNumber("Integrated Position", () -> module.getTurningPosition());
        layout.addNumber("Velocity", () -> module.getDriveVelocity());
        layout.withSize(2, 4);
    }

    private void updateShuffleboard(){
        
    }

 
}
