package org.firstinspires.ftc.teamcode.simulator.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.simulator.commnanda.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.simulator.subsystems.IntakeSubsystemSimulate;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

/**
 * Drives {@link MecanumDriveSubsystemSimulation} from gamepad 1 - no robot hardware required.
 * Wires up {@link DriveSimCommand} as the drive subsystem's default command, so it runs every
 * loop via the CommandScheduler (started by {@link #run()}) until something else takes over
 * the subsystem.
 */
@TeleOp(name = "Teleop (Simulated)", group = "Simulator")
@Disabled
public class TeleopSimulated extends CommandOpMode {

    private MecanumDriveSubsystemSimulation drive;

    private IntakeSubsystemSimulate intake;

    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        GlobalData.setRedAlliance();
        drive = new MecanumDriveSubsystemSimulation(this);
        intake = new IntakeSubsystemSimulate();
        driverGamepad = new GamepadEx(gamepad1);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(GlobalData.toggleAllianceCommand());
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileActiveOnce(intake.runIntakeCommand());
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveOnce(intake.stopIntakeCommand());

        drive.setDefaultCommand(new DriveSimCommand(
                drive,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX(), () -> true));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}
