package org.firstinspires.ftc.teamcode.simulator.subsystems;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

public class ShooterSubsystemSimulate extends SubsystemBase {
    private final MotorSimulator shooterMotor;

    private final TelemetryManager telemetryM;
    protected double simDt = 0.01; // 10ms fixed physics step (100Hz)

    public ShooterSubsystemSimulate() {
        shooterMotor = new MotorSimulator(2500);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void runShooter() {
        shooterMotor.setPower(Configurables.shooterPowerSim);
    }

    public Command runShooterCommand() {
        return new InstantCommand((this::runShooter));
    }

    public void stopShooter() {
        shooterMotor.setPower(0);
    }

    public Command stopShooterCommand() {
        return new InstantCommand((() -> shooterMotor.setPower(0)));
    }

    @Override
    public void periodic() {
        shooterMotor.update(shooterMotor.getPower(), simDt);
        if (Configurables.showShooterTelemetry)
            updateTelemetry();
    }

    public void updateTelemetry() {

        telemetryM.addData("ShooterPower", shooterMotor.getPower());
        telemetryM.addData("ShooterRPM", shooterMotor.getVelocityRPM());
       // telemetryM.update();
    }
}


