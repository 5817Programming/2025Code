package com.team5817.frc2025.subsystems.Rollers;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class RollerConstantsTalonFX {
    public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public double kKp = 0; // Raw output / raw error
    public double kKi = 0; // Raw output / sum of raw error
    public double kKd = 0; // Raw output / (err - prevErr)
    public double kKv = 0;
    public double kKa = 0;
    public double kKs = 0;

    public double kRampRate = 0.0; // s

    public int kSupplyCurrentLimit = 60; // amps
    public boolean kEnableSupplyCurrentLimit = false;

    public int kStatorCurrentLimit = 40; // amps
    public boolean kEnableStatorCurrentLimit = false;

    public double kMaxForwardOutput = 12.0; // Volts
    public double kMaxReverseOutput = -12.0; // Voltsa

    public boolean counterClockwisePositive = false;
}
