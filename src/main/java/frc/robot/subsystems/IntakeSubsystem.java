package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(5, "Drive Base");
    private final VelocityVoltage m_velocity = new VelocityVoltage(0);

    public IntakeSubsystem() {
        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.11;
        configs.Slot0.kV = 0.12;


        var status = intakeMotor.getConfigurator().apply(configs);
        if (!status.isOK()) {
            System.out.println("Could not configure Talon ID 5: " + status.toString());
        }
    }

    public void setVelocity(double rps) {
        intakeMotor.setControl(m_velocity.withVelocity(rps));
    }

    public Command runIntakeCommand(double rps) {
        return this.runEnd(
        () -> this.setVelocity(rps), 
        () -> this.intakeMotor.stopMotor());
    }

  public Command beginIntakeCommand() {
        return new InstantCommand(() -> this.setVelocity(30));
    }

    public Command endIntakeCommand() {
        return new InstantCommand(() -> this.intakeMotor.stopMotor());
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    } 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeVelocity", getIntakeVelocity());
    }
}
