// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {

    private SparkMax leftMotor, rightMotor;

    public Elevator() {
        this.leftMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_LEFT, MotorType.kBrushless);
        this.rightMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_RIGHT, MotorType.kBrushless);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.inverted(Constants.ElevatorConstants.leftInverted).idleMode(IdleMode.kBrake);
        leftConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        leftConfig.closedLoop.pid(
            Constants.ElevatorConstants.ELEVATOR_PID[0],
            Constants.ElevatorConstants.ELEVATOR_PID[1],
            Constants.ElevatorConstants.ELEVATOR_PID[2]
        );

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        rightConfig.closedLoop.pid(
            Constants.ElevatorConstants.ELEVATOR_PID[0],
            Constants.ElevatorConstants.ELEVATOR_PID[1],
            Constants.ElevatorConstants.ELEVATOR_PID[2]
        );
        rightConfig.follow(Constants.ElevatorConstants.ELEVATOR_LEFT, true); // Just clones the voltage so technically the above pid settings do nothing but nice to have!

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // basic motor methods
    public void set(double speed) {
        this.leftMotor.set(speed);
    }

    public void stop() {
        this.leftMotor.set(0);
    }

    public SparkMax getLeftMotor() {
        return this.leftMotor;
    }

    public SparkMax getRightMotor() {
        return this.rightMotor;
    }

    public void resetEncoders() {
        this.leftMotor.getEncoder().setPosition(0);
        this.rightMotor.getEncoder().setPosition(0);
    }

    public double getAvgEncoderTicks() {
        return (this.leftMotor.getEncoder().getPosition() + this.rightMotor.getEncoder().getPosition()) / 2.0;
    }

}