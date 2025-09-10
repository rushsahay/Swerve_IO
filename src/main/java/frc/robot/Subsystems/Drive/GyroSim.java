package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroSim implements GyroIO {
  private GyroSimulation gyro;

  public GyroSim(GyroSimulation gyro) {
    this.gyro = gyro;
  }

  @Override
  public void setYawDegrees(double yaw) {
    gyro.setRotation(Rotation2d.fromDegrees(yaw));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;

    inputs.xVelocity = 0;
    inputs.yVelocity = 0;

    inputs.xAcceleration = 0;
    inputs.yAcceleration = 0;
    inputs.zAcceleration = 0;

    inputs.resetOccured = false;

    inputs.supplyVoltage = 0;
    inputs.temperature = 0;

    inputs.yaw = gyro.getGyroReading().getDegrees();

    inputs.zVelocity = gyro.getMeasuredAngularVelocity().abs(RadiansPerSecond);
  }
}
