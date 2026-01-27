// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_turretTurner = new TalonFX(0, "Aux");
    private final TalonFX m_turretShooter = new TalonFX(5, "Aux");
    private final TalonFX m_turretFeeder = new TalonFX(3, "Aux");

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);

    public TurretSubsystem() {
        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.12;
        configs.Slot0.kV = 0.12;

        var turretTurnerStatus = m_turretTurner.getConfigurator().apply(configs);
        var turretShooterStatus = m_turretShooter.getConfigurator().apply(configs);
        var turretFeederStatus = m_turretFeeder.getConfigurator().apply(configs);


        if (!turretTurnerStatus.isOK()) {
            System.out.println("Could not configure Turret Turner Motor: " + turretTurnerStatus.toString());
        }
        if (!turretShooterStatus.isOK()) {
            System.out.println("Could not configure Turret Shooter Motor: " + turretShooterStatus.toString());
        }
        if (!turretFeederStatus.isOK()) {
            System.out.println("Could not configure Turret Feeder Motor: " + turretFeederStatus.toString());
        }
    }

    public void turnTurret(double speed) {
       m_turretTurner.setControl(m_velocity.withVelocity(speed * 2));
    }

    public Command shootTurret(double rps) {
        return this.runEnd(
          () -> {
          this.m_turretShooter.setControl(m_velocity.withVelocity(-rps)); 
          this.m_turretFeeder.setControl(m_velocity.withVelocity(rps));
          },
          () -> {
            this.m_turretShooter.stopMotor();
            this.m_turretFeeder.stopMotor();
          }
        );
      }
    

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
