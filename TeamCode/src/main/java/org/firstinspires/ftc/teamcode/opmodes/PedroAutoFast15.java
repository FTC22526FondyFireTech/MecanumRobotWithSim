package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Fast15;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

/**
 *
 * {@link MecanumDriveSubsystemSimulation} instead of real hardware - no robot required.
 * Swap {@link org.firstinspires.ftc.teamcode.utils.Constants#createFollower(com.qualcomm.robotcore.hardware.HardwareMap)}
 * for {@link SimulatorConstants#createSimulatedFollower(MecanumDriveSubsystemSimulation)} and
 * everything else (PathChains, FollowPathCommand, follower.update()/getPose()) works unchanged,
 * since both factories hand back a real {@code Follower}.
 */
@Autonomous(name = "Pedro Auto Fast 15", group = "Auto")
public class PedroAutoFast15 extends CommandOpMode {


    private MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;

    private IntakeSubsystem intake;

    private Follower follower;

    private Fast15 f15;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private final long pickupTime_ms = 2000;
    private final long scoreTime_ms = 3000;

    private int seqNum;


    @Override
    public void initialize() {
        super.reset();
        if (!Configurables.doSimulation)
            drive = new MecanumDriveSubsystem(this.hardwareMap);
        else
            driveSim = new MecanumDriveSubsystemSimulation(this);


        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
            follower = SimulatorConstants.createSimulatedFollower(driveSim);


        intake = new IntakeSubsystem(this.hardwareMap);

        GlobalData.selectAlliance(this);

        f15 = new Fast15();

        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/PathChain/FollowPathCommand usage.
        follower = SimulatorConstants.createSimulatedFollower(driveSim);

        f15.buildPaths(follower);

        follower.setStartingPose(f15.start);


        schedule(
                Commands.sequence(

                        new FollowPathCommand(follower, f15.scoreP),
                        new WaitCommand(scoreTime_ms),
                        //   intakeCommand(f15.intake1P),

                        new FollowPathCommand(follower, f15.score1P),
                        new WaitCommand(scoreTime_ms),
                        intakeCommand(f15.intake2P),

                        new FollowPathCommand(follower, f15.score2P),
                        new WaitCommand(scoreTime_ms),

                        intakeCommand(f15.intake3P),

                        new FollowPathCommand(follower, f15.score3P),

                        new WaitCommand(scoreTime_ms)));

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed) {
            run();
            follower.update();
            telemetryM.addData("Index", seqNum);
            telemetryM.addData("X", follower.getPose().getX());
            telemetryM.addData("Y", follower.getPose().getY());
            telemetryM.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetryM.addData("Busy", follower.isBusy());
            telemetryM.update(telemetry);
        }
        reset();

    }

    public Command intakeCommand(PathChain pc) {
        return
                Commands.sequence(

                        Commands.runOnce(() -> intake.runIntake()),
                        new FollowPathCommand(follower, pc),

                        new WaitCommand(pickupTime_ms),
                        intake.stopIntakeCommand());
    }
}