// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkMax;

import java.util.spi.CurrencyNameProvider;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlgaeClawConstants;

public class AlgaePivot extends SubsystemBase {

    private final SparkMax pivotMotor;
    private double setpointTicks = 0.0;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;

    public AlgaePivot() {
        this.pivotMotor = new SparkMax(Constants.AlgaeClawConstants.ALGAE_PIVOT_ID, MotorType.kBrushless);

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
        
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.pivotMotor.getEncoder().setPosition(0);
        this.goalState = new TrapezoidProfile.State(this.setpointTicks, 0);
    }

    public SparkMax getPivotMotor() {
        return this.pivotMotor;
    }

    public void setPivotMotor(double percentSpeed) {
        this.pivotMotor.set(percentSpeed);
    }

    public void setPivotPosition(double setpointTicks) {
        if (0 <= setpointTicks && setpointTicks <= Constants.AlgaeClawConstants.PIVOT_OUT_TICKS) {
            this.setpointTicks = setpointTicks;
        }
    }

    public double getPivotEncoderTicks() {
        return this.pivotMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(AlgaeClawConstants.MAX_VELOCITY, AlgaeClawConstants.MAX_ACCELERATION)
        );

        this.currentState = new TrapezoidProfile.State(this.getPivotEncoderTicks(), this.pivotMotor.getEncoder().getVelocity());
        this.goalState = new TrapezoidProfile.State(this.setpointTicks, 0);

        currentState = profile.calculate(0.02, currentState, goalState);

        this.pivotMotor.getClosedLoopController().setReference(
            this.goalState.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        SmartDashboard.putNumber("Pivot Setpoint", setpointTicks);
        SmartDashboard.putNumber("Pivot Current Position", currentState.position);
        SmartDashboard.putNumber("Pivot Error", this.pivotMotor.getClosedLoopController().getIAccum());
    }
}
