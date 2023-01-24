package frc.robot.util.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.vision.PoseEstimator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.SwerveSubsytem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.DriveConstants;

public class AlignToTarget extends CommandBase{

    private final PhotonCamera cam; 
    private final SwerveSubsytem swerve; 
    public AprilTagFieldLayout aprilTagFieldLayout;

    private static final Transform3d TAG_TO_GOAL = 
    new Transform3d(
        new Translation3d(CameraConstants.TARGET_RANGE, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI));

    private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(CameraConstants.MAX_ALIGN_VELOCITY, CameraConstants.MAX_ALIGN_ACCELERATION);
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(CameraConstants.MAX_ALIGN_VELOCITY, CameraConstants.MAX_ALIGN_ACCELERATION);
    private static final TrapezoidProfile.Constraints rotateConstraints = new TrapezoidProfile.Constraints(CameraConstants.MAX_ROTATE_VELOCITY, CameraConstants.MAX_ROTATE_ACCELERATION);

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints); //FIX 
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
    private final ProfiledPIDController rotateController = new ProfiledPIDController(2, 0, 0, rotateConstraints);


    /*
     * how to align to target 
     * have robotpose from estimator 
     * robotpose to desired pose using pid controllers w swerve 
     * 
     */

    public AlignToTarget(PhotonCamera cam, SwerveSubsytem swerve){
        this.cam = cam; 
        this.swerve = swerve; 

        //Gets Apriltag layout from JSON with IDs and poses.
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile); 
        } catch (IOException e){
            System.out.println(e);
        } 

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        rotateController.setTolerance(Units.degreesToRadians(3)); //FIX 
        rotateController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve); 
    }

    @Override
    public void initialize() {
        var robotPose = swerve.getCurrentPose();
        rotateController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }



    @Override
    public void execute() {
        var robotPose2d = swerve.getCurrentPose();
        var robotPose = 
            new Pose3d(robotPose2d);

        Pose2d goalPose = null;

        var result = cam.getLatestResult();
       
        if (result.hasTargets()){
            var target = result.getBestTarget();
            var targetId = target.getFiducialId();  //gets best apriltag ID 

            if (target.getPoseAmbiguity() <= 0.2 && targetId >= 0){
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(targetId);
                var targetPose = tagPose.get();     //gets apriltag pose 

                goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();     //gets goal pose from apriltag pose 
            }
        } 

        if(goalPose != null){
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            rotateController.setGoal(goalPose.getRotation().getRadians()); 
        }



        // Drive to the target
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var rotateSpeed = rotateController.calculate(robotPose2d.getRotation().getRadians());
        if (rotateController.atGoal()) {
            rotateSpeed = 0;
        }

        ChassisSpeeds speeds =ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, robotPose2d.getRotation());
        swerve.setChassisSpeeds(speeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }


    
}
