/* package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

public class PoseSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    
    private LimelightHelpers.PoseEstimate mt1; // we use megatag 1 for the yaw, since megatag 2 assumes you already have the yaw
    private LimelightHelpers.PoseEstimate mt2; // megatag 2 for translation, mt1 for rotation
    

    
    public PoseSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) { //constructor
        this.drivetrain = drivetrain; 
        this.limelightName = limelightName;
        
        
    }

    private Vector<N3> calculateStdDev(double distance) {
        double xyUncertainty = Math.max(0.05, 0.1 * Math.pow(distance, 2)); //there is Math.max because it'll spaz out if we dont give it a floor...
        double thetaUncertainty = Math.max(Units.degreesToRadians(2), Math.toRadians(40) * Math.pow(distance, 2));
        Vector<N3> calculatedStdDevs = VecBuilder.fill(xyUncertainty, xyUncertainty, thetaUncertainty); 

        return calculatedStdDevs;
    }

    private void setMT2Yaw() { 
        if (!LimelightHelpers.getTV(limelightName)) {
            return;
        }
        else {  
            LimelightHelpers.SetRobotOrientation(limelightName, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            }
        }
    
    private void estimatePose() {
        mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        // we add the re-definitions of megatags in order to update their bot pose estimates

        


        if (mt2.tagCount <= 0 || mt2 == null || mt1.tagCount <= 0 || mt1 == null) {
            return;
        }

        if (mt2.rawFiducials[0].distToRobot < 4) {
            setMT2Yaw();
            
            drivetrain.addVisionMeasurement(
                mt2.pose, 
                mt2.timestampSeconds, 
                calculateStdDev(mt2.avgTagDist)
            );
            //System.out.println(drivetrain.getState().Pose);
        }
    }

    @Override
    public void periodic() {
        estimatePose();
    }
}
*/

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;


public class PoseSubsystem extends SubsystemBase {
     final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain;
    private final List<String> limelightNames = List.of("limelight-fleft", "limelight-fright"); //include limelight-fright later on when you figure out how to get avgs between two limelights
    

    public PoseSubsystem(CommandSwerveDrivetrain drivetrain) { //constructor
        this.drivetrain = drivetrain;        
        SmartDashboard.putData(field); 
    }

    private Vector<N3> calculateStdDev(double distance) {
        double xyUncertainty = Math.max(0.1, 0.1 * Math.pow(distance, 2)); //there is Math.max because it'll spaz out if we dont give it a floor...
        double thetaUncertainty = Math.max(4, 4 * Math.pow(distance, 2));
        Vector<N3> calculatedStdDevs = VecBuilder.fill(xyUncertainty, xyUncertainty, thetaUncertainty); 

        return calculatedStdDevs;
    }

    private void updateVision(String name) {

        if (!LimelightHelpers.getTV(name)) {return;} //if limelight not exist, then dont even bother running the rest

        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        LimelightHelpers.SetRobotOrientation(name, mt1.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0); //gives MT2 the current rotation of the bot
        
        if ( mt1 == null || mt1.tagCount == 0 || 
             mt2 == null || mt2.tagCount == 0 || mt2.avgTagDist > 1.5) 
             {return;}

        Vector<N3> stdDevs = calculateStdDev(mt2.avgTagDist); //we need our std dev for rotation to be high so that the robot uses the gyro for drivetrain, but the stddev will nudge the gyro in the right direction (if it drifts)

        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }

    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    public double getDistToTarget(Translation2d targetPosition) {
        return drivetrain.getState().Pose.getTranslation().getDistance(targetPosition);
    }

    public void printCurrentPose(){
        System.out.println(drivetrain.getState().Pose);
    }
    
    @Override
    public void periodic() {
        for (String name : limelightNames) {
            updateVision(name);
        }
        //printCurrentPose();
        field.setRobotPose(drivetrain.getState().Pose);
    }


}
