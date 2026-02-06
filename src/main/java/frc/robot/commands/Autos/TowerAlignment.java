package frc.robot.commands.Autos;

import frc.robot.subsystems.PoseSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;



public class TowerAlignment{
    private final Pose2d towerPosition = new Pose2d(1.6, 3.75, Rotation2d.fromDegrees(0));
    private final PathConstraints constraints = new PathConstraints(3, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    //insert more positions for important places here


    public Command alignToTower() {
        final Command towerAlignCommand = AutoBuilder.pathfindToPose(
            towerPosition, 
            constraints,
            0.0);
        return towerAlignCommand;
    }

}
