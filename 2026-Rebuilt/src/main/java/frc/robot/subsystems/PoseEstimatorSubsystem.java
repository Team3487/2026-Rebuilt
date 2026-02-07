package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.LimelightConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class PoseEstimatorSubsystem extends SubsystemBase{

    Limelight limelightFront;
    Limelight limelightLeft;

    Pigeon2   gyro;

    boolean tooFast;

    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;

    PoseEstimate poseEstimate;

    public PoseEstimatorSubsystem(CommandSwerveDrivetrain MySillyLittleDrivetrain){
        limelightFront = new Limelight(LimelightConstants.LimelightFrontID);
        limelightLeft = new Limelight(LimelightConstants.LimelightLeftID);
        gyro = new Pigeon2(13);
        m_CommandSwerveDrivetrain = MySillyLittleDrivetrain;

        limelightFront.getSettings().withCameraOffset(LimelightConstants.limelightFrontPose).save();
        limelightLeft.getSettings().withCameraOffset(LimelightConstants.limelightLeftPose).save();

        
    }

    @Override
    public void periodic() {

        

        limelightFront.getSettings().withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
												 new AngularVelocity3d(DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble())))).save();
        limelightLeft.getSettings().withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
												 new AngularVelocity3d(DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble())))).save();
    Optional<PoseEstimate> visionEstimateFront = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    // If the pose is present
    visionEstimateFront.ifPresent((PoseEstimate poseEstimateFront) -> {
    // Add it to the pose estimator.
    m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateFront.pose.toPose2d(), poseEstimateFront.timestampSeconds);
    });


    Optional<PoseEstimate> visionEstimateLeft = limelightLeft.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    // If the pose is present
    visionEstimateLeft.ifPresent((PoseEstimate poseEstimateLeft) -> {
    // Add it to the pose estimator.
    m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateLeft.pose.toPose2d(), poseEstimateLeft.timestampSeconds);
    });
}
    


}