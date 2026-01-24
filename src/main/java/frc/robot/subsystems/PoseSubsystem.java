package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

public class PoseSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<String> limelightNames = List.of("limelight-fleft", "limelight-fright");
    

    public PoseSubsystem(CommandSwerveDrivetrain drivetrain) { //constructor
        this.drivetrain = drivetrain;         
    }

    @Override
    public void periodic() {
        for (String name : limelightNames) {
            updateVision(name);
        }
    }

    private void updateVision(String name) {

        if (!LimelightHelpers.getTV(name)) {return;} //if limelight not exist, then dont even bother running the rest

        double yawRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double gyroYaw = drivetrain.getState().RawHeading.getDegrees();

        LimelightHelpers.SetRobotOrientation(name, gyroYaw, yawRate, 0, 0, 0, 0); //gives MT2 the current rotation of the bot

        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 == null || mt2.tagCount == 0 || mt2.avgTagDist > 4.0) {return;}

        Vector<N3> stdDevs = VecBuilder.fill(0.1, 0.1, 10.0); //we need our std dev for rotation to be high so that the robot uses the gyro for drivetrain, but the stddev will nudge the gyro in the right direction (if it drifts)

        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }
}

