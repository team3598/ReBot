// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class TurretConstants {
    //motor declarations
    public static final TalonFX turretTurner = new TalonFX(0, "Aux");
    public static final TalonFX turretShooter = new TalonFX(5, "Aux");
    public static final TalonFX turretFeeder = new TalonFX(3, "Aux");
    public static final TalonFX turretHood = new TalonFX(1, "Aux");
    public static final TalonFX turretHopper = new TalonFX(2, "Aux");
    

    public static final CANcoder turretTurnerEncoder = new CANcoder(6, "Aux");
    public static final double turretTurnerOffset = 0.5; 
}
