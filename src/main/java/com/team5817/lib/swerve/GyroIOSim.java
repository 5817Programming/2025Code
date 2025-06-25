package com.team5817.lib.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.team254.lib.geometry.Rotation2d;
import com.team5817.lib.util.PhoenixUtil;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = new Rotation2d(gyroSimulation.getGyroReading());
        inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        List<Rotation2d> gyroReadings = new ArrayList<>();
        for(edu.wpi.first.math.geometry.Rotation2d wpi:gyroSimulation.getCachedGyroReadings()){
            gyroReadings.add(new Rotation2d(wpi));
        }
        inputs.odometryYawPositions = gyroReadings.toArray(new Rotation2d[0]);
    }
}