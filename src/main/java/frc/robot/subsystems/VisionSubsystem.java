package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) { //constructor
        this.drivetrain = drivetrain; 
        this.limelightName = limelightName;
    }

    private edu.wpi.first.math.Vector<N3> calculateStdDev(double distance) {
        double xyUncertainty = Math.max(0.05, 0.1 * Math.pow(distance, 2)); //there is Math.max because it'll spaz out if we dont give it a floor...
        double thetaUncertainty = Math.max(Units.degreesToRadians(2), Math.toRadians(40) * Math.pow(distance, 2));
        Vector<N3> calculatedStdDevs = VecBuilder.fill(xyUncertainty, xyUncertainty, thetaUncertainty); 

        return calculatedStdDevs;
    }

    @Override
    public void periodic() {
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    
    if (mt1.tagCount > 0) {
        double distance = drivetrain.getState().Pose.getTranslation().getDistance(mt1.pose.getTranslation());
        System.out.println(distance);
        
        if (distance < 100.0) { 
            drivetrain.addVisionMeasurement(
                mt1.pose, 
                mt1.timestampSeconds, 
                calculateStdDev(distance)
            );
            System.out.println(drivetrain.getState().Pose.getTranslation());
        }
    }
}
}
