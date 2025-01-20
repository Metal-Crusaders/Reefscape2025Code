// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants;

public class AlgaeClaw extends SubsystemBase {

    private SparkMax clawMotor, pivotMotor;
    private double setpointTicks = 0.0; // Internal setpoint for the elevator position
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;

    public AlgaeClaw() {
        this.clawMotor = new SparkMax(Constants.AlgaeClawConstants.ALGAE_CLAW_ID, MotorType.kBrushless);
        this.pivotMotor = new SparkMax(Constants.AlgaeClawConstants.ALGAE_PIVOT_ID, MotorType.kBrushless);

        SparkMaxConfig clawConfig = new SparkMaxConfig();
        clawConfig.inverted(Constants.AlgaeClawConstants.CLAW_INVERTED).idleMode(IdleMode.kBrake);
        clawConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        clawConfig.closedLoopRampRate(Constants.AlgaeClawConstants.RAMP_RATE);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.inverted(Constants.AlgaeClawConstants.PIVOT_INVERTED).idleMode(IdleMode.kBrake);
        pivotConfig.encoder.positionConversionFactor(420).velocityConversionFactor(420);
        pivotConfig.closedLoopRampRate(Constants.AlgaeClawConstants.RAMP_RATE);
        pivotConfig.closedLoop.pid(
            Constants.AlgaeClawConstants.PIVOT_PID[0],
            Constants.AlgaeClawConstants.PIVOT_PID[1],
            Constants.AlgaeClawConstants.PIVOT_PID[2],
            ClosedLoopSlot.kSlot0
        );
        
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SparkMax getClawMotor() {
        return this.clawMotor;
    }

    public SparkMax getPivotMotor() {
        return this.pivotMotor;
    }

    public void setClawMotor(double percentSpeed) {
        this.clawMotor.set(percentSpeed);
    }

    public void setPivotMotor(double percentSpeed) {
        this.setPivotMotor(percentSpeed);
    }

    public void setPivotPosition(double setpointTicks) {
        // Set the internal setpoint only
        if (0 <= setpointTicks && setpointTicks <= Constants.ElevatorConstants.MAX_ENCODER_TICKS) {
            this.setpointTicks = setpointTicks;
        }
    }

    public double getPivotEncoderTicks() {
        return this.pivotMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        // Create a TrapezoidProfile to calculate the next setpoint
        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
        );

        this.goalState = new TrapezoidProfile.State(this.setpointTicks, 0);

        // Update the current state with the profile's next state
        currentState = profile.calculate(0.02, goalState, currentState); // Assuming periodic runs every 20ms

        // Use the updated position to move the elevator
        this.pivotMotor.getClosedLoopController().setReference(
            currentState.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            Constants.AlgaeClawConstants.PIVOT_PID[4],
            ArbFFUnits.kVoltage
        );
    }

}
