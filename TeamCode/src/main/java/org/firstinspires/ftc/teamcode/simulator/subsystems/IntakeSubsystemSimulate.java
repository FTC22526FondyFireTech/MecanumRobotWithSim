package org.firstinspires.ftc.teamcode.simulator.subsystems;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

public class IntakeSubsystemSimulate extends SubsystemBase {
    private final MotorSimulator intakeMotor;

    private final TelemetryManager telemetryM;
    protected double simDt = 0.01; // 10ms fixed physics step (100Hz)

    public IntakeSubsystemSimulate() {
        intakeMotor = new MotorSimulator(1500);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void runIntake() {

        intakeMotor.setPower(Configurables.intakePowerSim);
    }

    public Command runIntakeCommand() {
        return new InstantCommand((this::runIntake));
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand((() -> intakeMotor.setPower(0)));
    }

    @Override
    public void periodic() {
        intakeMotor.update(intakeMotor.getPower(), simDt);
        if (Configurables.showIntakeTelemetry)
            updateTelemetry();
    }

    public void updateTelemetry() {

        telemetryM.addData("IntakePower", intakeMotor.getPower());
        telemetryM.addData("IntakeRPM", intakeMotor.getVelocityRPM());
    }
}


