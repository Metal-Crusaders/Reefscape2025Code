// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.spi.CurrencyNameProvider;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private SparkMax leftMotor, rightMotor;
    private final SysIdRoutine sysIdRoutine;
    private double setpointTicks = 0.0; // Internal setpoint for the elevator position
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile profile;

    private double prevUpdateTime = Timer.getFPGATimestamp();

    public Elevator() {
        this.leftMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_LEFT, MotorType.kBrushless);
        this.rightMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_RIGHT, MotorType.kBrushless);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.inverted(Constants.ElevatorConstants.LEFT_INVERTED).idleMode(IdleMode.kBrake);
        leftConfig.closedLoop.pid(
            Constants.ElevatorConstants.ELEVATOR_PID[0],
            Constants.ElevatorConstants.ELEVATOR_PID[1],
            Constants.ElevatorConstants.ELEVATOR_PID[2],
            ClosedLoopSlot.kSlot0
        ).maxOutput(ElevatorConstants.MAX_PERCENT_SPEED)
        .minOutput(-1 * ElevatorConstants.MAX_PERCENT_SPEED);
        // leftConfig.closedLoop.maxMotion
        //     .maxAcceleration(Constants.ElevatorConstants.MAX_ACCELERATION)
        //     .maxVelocity(Constants.ElevatorConstants.MAX_VELOCITY)
        //     .allowedClosedLoopError(0.1);
        leftConfig.closedLoopRampRate(ElevatorConstants.RAMP_RATE);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.closedLoopRampRate(ElevatorConstants.RAMP_RATE);
        rightConfig.follow(Constants.ElevatorConstants.ELEVATOR_LEFT, true); // Just clones the voltage so technically the above pid settings do nothing even if we set them here!
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize TrapezoidProfile states
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        // Initialize SysIdRoutine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), // Default config
            new SysIdRoutine.Mechanism(
                voltage -> leftMotor.setVoltage(voltage), // Apply voltage to motor
                null, // No feedforward needed
                this
            )
        );

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
        );



        resetEncoders();
    }

    // Basic motor methods
    public void setSpeed(double speed) {
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
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
    }

    public double getAvgEncoderTicks() {
        return (this.leftMotor.getEncoder().getPosition() + this.rightMotor.getEncoder().getPosition()) / 2.0;
    }

    public void setElevatorPosition(double setpointTicks) {
        // Set the internal setpoint only
        if (0 <= setpointTicks && setpointTicks <= Constants.ElevatorConstants.MAX_ENCODER_TICKS) {
            this.setpointTicks = setpointTicks;
        }
    }

    @Override
    public void periodic() {

        this.goalState = new TrapezoidProfile.State(this.setpointTicks, 0);

        // Update the current state with the profile's next state
        currentState = profile.calculate(Timer.getFPGATimestamp() - prevUpdateTime, currentState, goalState); // Assuming periodic runs every 20ms

        prevUpdateTime = Timer.getFPGATimestamp();

        // Use the updated position to move the elevator
        this.leftMotor.getClosedLoopController().setReference(
            goalState.position, // TESTING THIS
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ElevatorConstants.ELEVATOR_PID[3],
            ArbFFUnits.kVoltage
        );

        SmartDashboard.putNumber("Current State Ticks", currentState.position);
        SmartDashboard.putNumber("Current State Velocity", currentState.velocity);
        SmartDashboard.putNumber("Goal State Ticks", goalState.position);
        SmartDashboard.putNumber("Current Ticks", this.getAvgEncoderTicks());
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

}
