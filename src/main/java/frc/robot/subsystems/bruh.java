// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class bruh extends SubsystemBase {
    private final TalonFX motor;

    private static final int CAN_ID = 13;

    private static final int FWD_LIMIT_SWITCH_PIN = 8;
    private static final int REV_LIMIT_SWITCH_PIN = 9;

    private final DigitalInput fwdLimitSwitch, revLimitSwitch;

    /* Configs */
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(80.);

    public enum ClimberPositions {
        CLIMB(8.),
        READY(124.);

        public double Position;

        private ClimberPositions(double position) {
            this.Position = position;
        }
    }

    public bruh() {
        /* Setup */
        motor = new TalonFX(CAN_ID);

        fwdLimitSwitch = new DigitalInput(FWD_LIMIT_SWITCH_PIN);
        revLimitSwitch = new DigitalInput(REV_LIMIT_SWITCH_PIN);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);

        /* ======= */
        /* CONFIGS */
        /* ======= */
        motor.getConfigurator().apply(currentConfigs);

        /* CAN Bus */
        motor.getVelocity().setUpdateFrequency(20.);
        motor.getPosition().setUpdateFrequency(20.);
        motor.getStatorCurrent().setUpdateFrequency(20.);
        motor.getDutyCycle().setUpdateFrequency(20.);
        motor.optimizeBusUtilization();
    }

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus)).unless(() -> {
            if (vBus > 0.)
                return fwdLimitSwitch.get();
            if (vBus < 0.)
                return revLimitSwitch.get();
            return false;
        });
    }

    public void stop() {
        motor.set(0);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public boolean forwardLimit() {
        return fwdLimitSwitch.get() && motor.getMotorVoltage().getValueAsDouble() > 0.2;
    }

    public boolean reverseLimit() {
        return revLimitSwitch.get() && motor.getMotorVoltage().getValueAsDouble() < -0.2;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("FWD LIMIT", fwdLimitSwitch.get());
        SmartDashboard.putBoolean("REV LIMIT", revLimitSwitch.get());

    }
}
