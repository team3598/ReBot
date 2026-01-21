package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
>>>>>>> 894cc0609d061f7b012e42392a06e3196715dff9
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

<<<<<<< HEAD
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

=======
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
    }

>>>>>>> 894cc0609d061f7b012e42392a06e3196715dff9
    @Override
    public void periodic() {
        double[] rawBotpose = NetworkTableInstance.getDefault()
            .getTable(limelightName) //insert limelight name here
            .getEntry("botpose_wpiblue") 
            .getDoubleArray(new double[0]);

        if (rawBotpose.length >= 7) { //checks if april tag, array will have 7+ items if yes
            double x = rawBotpose[0];
            double y = rawBotpose[1];
<<<<<<< HEAD
            double degrees = rawBotpose[5]; //rotation

            double latency = rawBotpose[6]/1000; //the time that the pic was taken on the limelight, in seconds. rawBotpose[6] is in milliseconds, but since our WPIlibs Timer uses seconds, we divide it by 1000 so the calculations are right
            double currentTime = Timer.getFPGATimestamp();

            Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
            double distance = drivetrain.getState().Pose.getTranslation().getDistance(visionPose.getTranslation()); //distance
            Vector<N3> stdDevs = calculateStdDev(distance);

            if (distance < 1.5) {
                double timestamp = currentTime - latency;
                drivetrain.addVisionMeasurement(visionPose, timestamp, stdDevs);
            }
    }
}
=======
            double degrees = rawBotpose[5];
            double latencyMs = rawBotpose[6];

            Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
            double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0); //because it'll correct itself thinking that the current visionPose is based off the current time, but it isnt. prevents spazzing

        drivetrain.addVisionMeasurement(visionPose, timestamp);
    }
}

>>>>>>> 894cc0609d061f7b012e42392a06e3196715dff9
}
