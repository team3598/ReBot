package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
    }

    @Override
    public void periodic() {
        double[] rawBotpose = NetworkTableInstance.getDefault()
            .getTable(limelightName) //insert limelight name here
            .getEntry("botpose_wpiblue") 
            .getDoubleArray(new double[0]);

        if (rawBotpose.length >= 7) { //checks if april tag, array will have 7+ items if yes
            double x = rawBotpose[0];
            double y = rawBotpose[1];
            double degrees = rawBotpose[5];
            double latencyMs = rawBotpose[6];

            Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
            double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0); //because it'll correct itself thinking that the current visionPose is based off the current time, but it isnt. prevents spazzing

        drivetrain.addVisionMeasurement(visionPose, timestamp);
    }
}

}
