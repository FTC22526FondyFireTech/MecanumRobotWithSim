package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commnands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;

//@Autonomous(name = "Blank")
@TeleOp(name = "Path Test")
//@Disabled

public class PathTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;
    //IntakeSubsystem intake;

    private Pose blueStartPose = new Pose(12, 12, Math.PI / 2);
    private Pose blueTestPose1 = new Pose(36, 48, Math.PI / 2);
    private Pose blueTestPose2 = new Pose(60, 84, Math.PI / 2);

    private Pose startPose = new Pose();
    private Pose testPose1 = new Pose();
    private Pose testPose2 = new Pose();

    private PathChain testChain1;


    private Follower follower;


    @Override
    public void initialize() {
        GlobalData.setBlueAlliance();
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


        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
            follower = SimulatorConstants.createSimulatedFollower(driveSim);

        setAlliancePaths();

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(
                Commands.sequence(
                        GlobalData.toggleAllianceCommand(),
                        Commands.runOnce(this::setAlliancePaths)));

        if (!Configurables.doSimulation) {
            driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whileActiveOnce(drive.resetPoseCommand());
        } else {
            driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whileActiveOnce(driveSim.getOdometry().resetPoseCommand());

        }

        if (!Configurables.doSimulation) {
            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whileActiveOnce(
                            Commands.defer(() ->
                                    drive.setPoseCommand(startPose), Arrays.asList(drive)));
        } else {
            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whileActiveOnce(
                            Commands.defer(() ->
                                    driveSim.getOdometry().setPoseCommand(startPose), Arrays.asList(driveSim)));

        }


        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileActiveOnce(
                        Commands.defer
                                (() -> Commands.sequence(
                                        Commands.runOnce(() -> follower.setStartingPose(startPose)),
                                        new FollowPathCommand(follower, testChain1, false)), Arrays.asList()));


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


    }

    private void setAlliancePaths() {
        if (GlobalData.isRedAlliance()) {
            startPose = flipBlueToRedPose(blueStartPose);
            testPose1 = flipBlueToRedPose(blueTestPose1);
            testPose2 = flipBlueToRedPose(blueTestPose2);

        } else {
            startPose = blueStartPose;
            testPose1 = blueTestPose1;
            testPose2 = blueTestPose2;
        }


        buildPaths();
    }


    public Pose flipBlueToRedPose(Pose blue) {
        double x = blue.getX();
        double y = blue.getY();
        x = SimulatorConstants.width - x;
        double h = blue.getHeading();
        return new Pose(x, y, Math.PI - h);
    }

    private void buildPaths() {

        testChain1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                testPose1,
                                testPose2)
                )
                .setLinearHeadingInterpolation(testPose1.getHeading(), testPose2.getHeading())

                .build();


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

            if (Configurables.doSimulation) follower.update();



            telemetryM.update(telemetry);
        }
        reset();
    }

}