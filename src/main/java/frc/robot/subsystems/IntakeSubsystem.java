package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intake = new TalonFX(4, "Aux");

    private final TalonFX m_intakeVL = new TalonFX(51, "Aux"); //VL stands for verticality left
    private final TalonFX m_intakeVR = new TalonFX(52, "Aux"); //VR stands for verticality right, this is the follower motor of VL

    private final Follower m_followRequest = new Follower(6, MotorAlignmentValue.Opposed);

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);

    public IntakeSubsystem() {
        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.12;
        configs.Slot0.kV = 0.12;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var intake_status = m_intake.getConfigurator().apply(configs);
        var intakeV_status = m_intakeVL.getConfigurator().apply(configs);

        if (!intake_status.isOK()) {
            System.out.println("Could not configure Intake Motor: " + intake_status.toString());
        }

         if (!intakeV_status.isOK()) {
            System.out.println("Could not configure Intake Verticality Motors: " + intakeV_status.toString());
        }

        m_intakeVL.setPosition(0);
    }

    public void setIntakeVelocity(double rps) {
        m_intake.setControl(m_velocity.withVelocity(rps));
    }

    public void setIntakeVerticality(double angle) {
        //insert intake verticality code here
    }

    public Command runIntakeCommand(double rps) {
        return this.runEnd(
        () -> this.setIntakeVelocity(30), 
        () -> this.m_intake.stopMotor());
    }

    public Command beginIntakeCommand() {
        return new InstantCommand(() -> this.setIntakeVelocity(30));
    }

    public Command endIntakeCommand() {
        return new InstantCommand(() -> this.m_intake.stopMotor());
    }

    public double getIntakeVelocity() {
        return m_intake.getVelocity().getValueAsDouble();
    } 

    @Override
    public void periodic() {
        m_intakeVR.setControl(m_followRequest);
        SmartDashboard.putNumber("IntakeVelocity", getIntakeVelocity());
    }
}
