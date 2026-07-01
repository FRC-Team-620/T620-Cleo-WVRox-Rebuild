// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for boron. */
public class GyroIOBoron implements GyroIO {
  private final Canandgyro boron;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOBoron() {
    // yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    // yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(this::getYAWDegrees);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this::getYAWDegrees);

    /** Code example from docs */

    // Creates a Canandgyro object referencing a Canandgyro with CAN ID 0
    this.boron = new Canandgyro(TunerConstants.kPigeonId);

    // Starts calibrating the gyroscope asynchronously
    this.boron.startCalibration();
  }

  private double getYAWDegrees() {
    return boron.getYaw() * 360.0;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = boron.isConnected();
    inputs.calibrated = !this.boron.isCalibrating();
    inputs.yawPosition = Rotation2d.fromDegrees(this.getYAWDegrees());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(boron.getAngularVelocityYaw());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
