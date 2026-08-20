package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.simulator.commnands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.purepursuit.Path;

//@Autonomous(name = "Blank")
@TeleOp(name = "Teleop")
//@Disabled

public class TeleopOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;
    //IntakeSubsystem intake;

    private Pose startPose = new Pose(12, 12, Math.PI / 2);

    private Pose testPose1 = new Pose(36, 12, Math.PI / 2);
    private Pose testPose2 = new Pose(60, 84, Math.PI / 2);
    private PathChain testChain1;

    @Override
    public void initialize() {
        GlobalData.setRedAlliance();
        driverGamepad = new GamepadEx(gamepad1);
        if (!Configurables.doSimulation) {
            drive = new MecanumDriveSubsystem(this.hardwareMap, new Pose());
            drive.setDefaultCommand(new DriveCommand(drive,
                    () -> driverGamepad.getLeftY(),
                    () -> -driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX()));
        } else {
            driveSim = new MecanumDriveSubsystemSimulation(this);
            driveSim.setDefaultCommand(new DriveSimCommand(
                    driveSim,
                    () -> driverGamepad.getLeftY(),
                    () -> driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX(), () -> true));
        }

        drive.getFollower().setStartingPose(startPose);

        testChain1 = drive.getFollower().pathBuilder()
                .addPath(
                        new BezierLine(
                                testPose1,
                                testPose2)
                )
                //.setTangentHeadingInterpolation()
                .build();

        //intake = new IntakeSubsystem(hardwareMap);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(GlobalData.toggleAllianceCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whileActiveOnce(drive.resetPoseCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileActiveOnce(drive.setPoseCommand(new Pose(12,12, Math.PI / 2)));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileActiveOnce(new FollowPathCommand(drive.getFollower(), testChain1));



        /*driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(intake.runIntakeCommand())
                .whenReleased(intake.stopIntakeCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(intake.invertIntakeCommand());

         */


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            if (!Configurables.doSimulation) {
                drive.setRobotCentric(driverGamepad.getGamepadButton(
                        GamepadKeys.Button.RIGHT_BUMPER).get());
                drive.showTelemetry(telemetryM);
            } else {
                driveSim.setRobotCentric(driverGamepad.getGamepadButton(
                        GamepadKeys.Button.RIGHT_BUMPER).get());
                driveSim.showTelemetry(telemetryM);
            }

            //intake.showTelemetry(telemetryM);

            telemetryM.update(telemetry);
        }
        reset();
    }

}