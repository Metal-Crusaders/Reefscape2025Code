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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CoralShooterConstants;

public class AlgaeClaw extends SubsystemBase {

    private SparkMax clawMotor, pivotMotor;

    private DigitalInput beamSensor;
    private DigitalOutput beam;

    public AlgaeClaw() {
        this.clawMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_LEFT, MotorType.kBrushless);
        this.pivotMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_RIGHT, MotorType.kBrushless);

        SparkMaxConfig clawConfig = new SparkMaxConfig();
        clawConfig.inverted(Constants.ElevatorConstants.leftInverted).idleMode(IdleMode.kBrake);
        clawConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        clawConfig.closedLoopRampRate(CoralShooterConstants.RAMP_RATE);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.inverted(!Constants.ElevatorConstants.leftInverted).idleMode(IdleMode.kBrake);
        rightConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        rightConfig.closedLoopRampRate(CoralShooterConstants.RAMP_RATE);
        
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // beam break
        this.beam = new DigitalOutput(CoralShooterConstants.CORAL_BEAM_ID);
        this.beam.set(true);
        this.beamSensor = new DigitalInput(CoralShooterConstants.CORAL_BEAM_SENSOR_ID);
    }

    public void stop() {
        this.clawMotor.set(0);
        this.pivotMotor.set(0);
    }

    public SparkMax getClawMotor() {
        return this.clawMotor;
    }

    public SparkMax getPivotMotor() {
        return this.pivotMotor;
    }

    public void set(double left, double right) {
        // sets percentage speed for now, TODO change to velocity PID!
        this.clawMotor.set(left);
        this.pivotMotor.set(right);
    }

    public boolean beamExists() {
        return beamSensor.get();
    }

}
