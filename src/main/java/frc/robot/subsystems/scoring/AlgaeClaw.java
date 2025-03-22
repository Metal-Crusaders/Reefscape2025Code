// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

public class AlgaeClaw extends SubsystemBase {

    private static final double PULSE_FREQUENCY = 1;
    private static final double PULSE_TIME = 0.2;
    private static final double PULSE_SPEED = -0.4;

    private static final double PASSIVE_HOLD = -0.1;

    public boolean currentlyUsed = false;
    
    private final SparkMax clawMotor;
    private final ColorSensorV3 colorSensor;
    private final Timer timer;

    public AlgaeClaw() {
        this.clawMotor = new SparkMax(Constants.AlgaeClawConstants.ALGAE_CLAW_ID, MotorType.kBrushless);

        SparkMaxConfig clawConfig = new SparkMaxConfig();
        clawConfig.inverted(Constants.AlgaeClawConstants.CLAW_INVERTED).idleMode(IdleMode.kBrake);
        clawConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        clawConfig.closedLoopRampRate(Constants.AlgaeClawConstants.RAMP_RATE);
        
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        this.timer = new Timer();
        this.timer.reset();
        this.timer.start();

        currentlyUsed = false;
    }

    public SparkMax getClawMotor() {
        return this.clawMotor;
    }

    public void setClawMotor(double percentSpeed) {
        this.clawMotor.set(percentSpeed);
    }

    public boolean holdingAlgae() {
        return this.colorSensor.getProximity() > Constants.AlgaeClawConstants.PROXIMITY_THRESHOLD;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Claw Motor Speed", clawMotor.get());
        // SmartDashboard.putNumber("Color Sensor Proximity", this.colorSensor.getProximity());
        // SmartDashboard.putNumber("Color Sensor Blue", this.colorSensor.getBlue());
        // SmartDashboard.putNumber("Color Sensor Test", this.colorSensor.getRed());

        // if (this.timer.hasElapsed(PULSE_FREQUENCY) && this.getClawMotor().get() == 0.0 && this.holdingAlgae()) {
        //     CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
        //         this.clawMotor.set(PULSE_SPEED);
        //         Timer.delay(PULSE_TIME);
        //         this.clawMotor.set(0.0);
        //     }));
        //     this.timer.reset();
        //     this.timer.start();
        // }

        if (!currentlyUsed) {
            if (this.holdingAlgae() && (this.clawMotor.get() == 0.0)) {
                this.clawMotor.set(PASSIVE_HOLD);
            } else {
                this.clawMotor.set(0);
            }
        }
    }
}
