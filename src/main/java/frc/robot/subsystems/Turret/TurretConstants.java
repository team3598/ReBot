// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class TurretConstants {
    //motor declarations
    public static final TalonFX turretTurner = new TalonFX(0, "Aux");
    public static final TalonFX turretShooter = new TalonFX(5, "Aux");
    public static final TalonFX turretFeeder = new TalonFX(3, "Aux");
    public static final TalonFX turretHood = new TalonFX(1, "Aux");
    public static final TalonFX turretHopper = new TalonFX(2, "Aux");
    public static final AnalogPotentiometer channel0 = new AnalogPotentiometer(0);
    public static final AnalogPotentiometer channel1 = new AnalogPotentiometer(1);
    public static final AnalogPotentiometer channel2 = new AnalogPotentiometer(2);
    public static final AnalogPotentiometer channel3 = new AnalogPotentiometer(3);
    public static final AnalogPotentiometer channel4 = new AnalogPotentiometer(4);
    public static final AnalogPotentiometer channel5 = new AnalogPotentiometer(5);
    public static final AnalogPotentiometer channel6 = new AnalogPotentiometer(6);
    public static final AnalogPotentiometer channel7 = new AnalogPotentiometer(7);



    public static final double pOffset = 0.000339498078;

    //temporary
    public static final CANcoder turretTurnerEncoder = new CANcoder(6, "Aux");
    public static final double turretTurnerOffset = 0.5; 
}
