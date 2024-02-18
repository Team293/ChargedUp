// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class DriveIOTalonFX implements DriveIO {
  private static final double GEAR_RATIO = 9.52;
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.16;
  private static final double kV = 0.113;

  private final TalonFX leftLeader = new TalonFX(1);
  private final TalonFX leftFollower = new TalonFX(3);
  private final TalonFX rightLeader = new TalonFX(0);
  private final TalonFX rightFollower = new TalonFX(2);
  private final StatusSignal<Double> leftPosition = leftLeader.getPosition();
  private final StatusSignal<Double> leftVelocity = leftLeader.getVelocity();
  private final StatusSignal<Double> leftAppliedVolts = leftLeader.getMotorVoltage();
  private final StatusSignal<Double> leftLeaderCurrent = leftLeader.getStatorCurrent();
  private final StatusSignal<Double> leftFollowerCurrent = leftFollower.getStatorCurrent();

  private final StatusSignal<Double> rightPosition = rightLeader.getPosition();
  private final StatusSignal<Double> rightVelocity = rightLeader.getVelocity();
  private final StatusSignal<Double> rightAppliedVolts = rightLeader.getMotorVoltage();
  private final StatusSignal<Double> rightLeaderCurrent = rightLeader.getStatorCurrent();
  private final StatusSignal<Double> rightFollowerCurrent = rightFollower.getStatorCurrent();

  private final Pigeon2 pigeon = new Pigeon2(20);
  private final StatusSignal<Double> yaw = pigeon.getYaw();

  public DriveIOTalonFX() {
    var leftConfig = new TalonFXConfiguration();
    leftConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.Slot0.kP = kP;
    leftConfig.Slot0.kI = kI;
    leftConfig.Slot0.kD = kD;
    leftConfig.Slot0.kS = kS;
    leftConfig.Slot0.kV = kV;
    leftLeader.getConfigurator().apply(leftConfig);
    leftFollower.getConfigurator().apply(leftConfig);

    var rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.Slot0.kP = kP;
    rightConfig.Slot0.kI = kI;
    rightConfig.Slot0.kD = kD;
    rightConfig.Slot0.kS = kS;
    rightConfig.Slot0.kV = kV;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightLeader.getConfigurator().apply(rightConfig);
    rightFollower.getConfigurator().apply(rightConfig);

    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leftPosition, rightPosition, yaw); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);
    leftLeader.optimizeBusUtilization();
    leftFollower.optimizeBusUtilization();
    rightLeader.optimizeBusUtilization();
    rightFollower.optimizeBusUtilization();
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent,
        yaw);

    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps =
        new double[] {leftLeaderCurrent.getValueAsDouble(), leftFollowerCurrent.getValueAsDouble()};

    inputs.rightPositionRad =
        Units.rotationsToRadians(rightPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps =
        new double[] {
          rightLeaderCurrent.getValueAsDouble(), rightFollowerCurrent.getValueAsDouble()
        };

    inputs.gyroYaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setControl(new VoltageOut(leftVolts));
    rightLeader.setControl(new VoltageOut(rightVolts));
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    leftLeader.setControl(
        new VelocityVoltage(Units.radiansToRotations(leftRadPerSec * GEAR_RATIO))
            .withSlot(0)
            .withEnableFOC(true));
    rightLeader.setControl(
        new VelocityVoltage(Units.radiansToRotations(rightRadPerSec * GEAR_RATIO))
            .withSlot(0)
            .withEnableFOC(true));
  }
}
