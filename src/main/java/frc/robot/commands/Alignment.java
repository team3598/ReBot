package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;


public class Alignment extends Command {
  private PIDController rotController;
  private double xController, yController;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivetrain;
  private double tagID = -1;
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 1;  
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
  private final SwerveRequest.RobotCentric drive_request = new SwerveRequest.RobotCentric();
  private final CommandXboxController joystick = new CommandXboxController(0);
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed



  public Alignment(CommandSwerveDrivetrain drivetrain) {
    xController = -joystick.getLeftY();
    yController = -joystick.getLeftX();  // Horizontal movement
    rotController = new PIDController(0.025, 0.01, 0); 
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

@Override
public void initialize() {
    rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-fright");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-fright") && LimelightHelpers.getFiducialID("limelight-fright") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-fright");
      SmartDashboard.putNumber("x", positions[2]);

      double rotValue = -rotController.calculate(positions[4]);
      
      
      drivetrain.setControl(drive_request.withVelocityX(xController * MaxSpeed).withVelocityY(yController * MaxSpeed).withRotationalRate(rotValue));

      System.out.println("rot velocity output: " + rotValue);

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
}
}