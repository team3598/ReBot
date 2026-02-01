// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretTurner = TurretConstants.turretTurner;
    private final TalonFX turretShooter = TurretConstants.turretShooter;
    private final TalonFX turretFeeder = TurretConstants.turretFeeder;
    private final TalonFX turretHood = TurretConstants.turretHood;
    private final TalonFX turretHopper = TurretConstants.turretHopper;
    private final CANcoder turretTurnerEncoder = TurretConstants.turretTurnerEncoder;

    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage turnerMMRequest = new MotionMagicVoltage(0); 
    private final MotionMagicVoltage hoodMMRequest = new MotionMagicVoltage(0); 

    public TurretSubsystem() {
        var configs = new TalonFXConfiguration();
        /*configs.Slot0.kP = 1.75;
        configs.Slot0.kD = 0.05;*/
        configs.Slot0.kP = 0.52;
        configs.Slot0.kV = 0.18; //0.138
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Feedback.SensorToMechanismRatio = 41.66666; //placeholder, change this
        turnerConfig.MotionMagic.MotionMagicCruiseVelocity = 0.75; 
        turnerConfig.MotionMagic.MotionMagicAcceleration = 1.0;  
        turnerConfig.MotionMagic.MotionMagicJerk = 10.0;         
        turnerConfig.Slot0.kP = 12; 
        turnerConfig.Slot0.kV = 0.1; 
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; 
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.05538; 

        final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Feedback.SensorToMechanismRatio = 250.0; 
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 0.75; 
        hoodConfig.MotionMagic.MotionMagicAcceleration = 1.0;  
        hoodConfig.MotionMagic.MotionMagicJerk = 10.0;         
        hoodConfig.Slot0.kP = 60; 
        hoodConfig.Slot0.kD = 0.1; 
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.25; 

        var turretTurnerStatus = turretTurner.getConfigurator().apply(turnerConfig);
        var turretShooterStatus = turretShooter.getConfigurator().apply(configs);
        var turretFeederStatus = turretFeeder.getConfigurator().apply(configs);
        var turretHoodStatus = turretHood.getConfigurator().apply(hoodConfig);
        var turretHopperStatus = turretHopper.getConfigurator().apply(configs);

        if (!turretTurnerStatus.isOK()) {
            System.out.println("Could not configure Turret Turner Motor: " + turretTurnerStatus.toString());
        }
        if (!turretShooterStatus.isOK()) {
            System.out.println("Could not configure Turret Shooter Motor: " + turretShooterStatus.toString());
        }
        if (!turretFeederStatus.isOK()) {
            System.out.println("Could not configure Turret Feeder Motor: " + turretFeederStatus.toString());
        }
        if (!turretHoodStatus.isOK()) {
            System.out.println("Could not configure Turret Hood Motor: " + turretHoodStatus.toString());
        }

        resetTurretAngle();
        turretHood.setPosition(0); //the actual angle irl is 20, 20/360
    }

    public void resetTurretAngle() {
        double absolutePosition = turretTurnerEncoder.getAbsolutePosition().getValueAsDouble();

        double calibratedPosition = absolutePosition - TurretConstants.turretTurnerOffset;

        turretTurner.setPosition(calibratedPosition);
    }

    public boolean isShooterAtSpeed(double targetRPS) {
        return Math.abs(turretShooter.getVelocity().getValueAsDouble() - targetRPS) < 1;
    }

    public void moveTurretAngle(double turretRotations) {
        turretTurner.setControl(turnerMMRequest.withPosition(turretRotations));
    }

    public void aimAtTarget(double distance) {
        //insert logic code here
        //turret angle = arctan(distanceFromTarget) - headingOfRobot
    }
    
    public void stopMotors(){
        turretTurner.stopMotor();
        turretHood.stopMotor();
        turretShooter.stopMotor();
        turretFeeder.stopMotor();
    }

    public void setShooterVelocity(double rps) {
        turretShooter.setControl(velocity.withVelocity(rps));
    }

    public void setFeederVelocity(double rps) {
        turretFeeder.setControl(velocity.withVelocity(rps));
    }

    public void setHoodAngle(double degrees) {
        turretHood.setControl(hoodMMRequest.withPosition(degrees/360));
    }

    public void setHopperSpeed(double rps) {
        turretHopper.setControl(velocity.withVelocity(rps));
    }
    
    public double getFeederSpeed(){
        return turretFeeder.getVelocity().getValueAsDouble();
    }

    public double getFlywheelSpeed(){
        return turretShooter.getVelocity().getValueAsDouble();
    }

    public Command goToHoodAngle(double degrees) {
       return this.runEnd(
        () -> {
                setHoodAngle(degrees);
                System.out.println("hood on. desired degrees of hood: " + degrees);
        },
        () -> turretHood.stopMotor()
        );
    }
    
    public Command shootTurret(double rps) {
        return this.runOnce(
            () -> {turretShooter.setControl(velocity.withVelocity(rps));}
        ).andThen(
            waitUntil(()->isShooterAtSpeed(rps))
        ).andThen(
            this.run(()->turretFeeder.setControl(velocity.withVelocity(100)))
        ).finallyDo(
            (interrupted)->{
            turretShooter.stopMotor();
            turretFeeder.stopMotor();
            }
        );
      }

    public Command goToAngle(double degrees) {
        // Factory that creates a command to move to a specific angle
        return this.runEnd(
            () -> moveTurretAngle(degrees / 360.0),
            () -> turretTurner.stopMotor()
        );
    } 

    @Override
    public void periodic() {
        //include a print statement for absolute encoder offset, then set that later.
        //System.out.println(turretShooter.getVelocity());
        //System.out.println(turretHood.getPosition());
        //System.out.println(turretTurner.getPosition());
    // This method will be called once per scheduler run
    }
}
