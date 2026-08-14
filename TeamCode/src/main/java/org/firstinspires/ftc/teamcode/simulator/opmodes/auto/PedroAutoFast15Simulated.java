package org.firstinspires.ftc.teamcode.simulator.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.simulator.subsystems.IntakeSubsystemSimulate;
import org.firstinspires.ftc.teamcode.simulator.subsystems.ShooterSubsystemSimulate;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

/**
 *
 * {@link MecanumDriveSubsystemSimulation} instead of real hardware - no robot required.
 * Swap {@link org.firstinspires.ftc.teamcode.utils.Constants#createFollower(com.qualcomm.robotcore.hardware.HardwareMap)}
 * for {@link SimulatorConstants#createSimulatedFollower(MecanumDriveSubsystemSimulation)} and
 * everything else (PathChains, FollowPathCommand, follower.update()/getPose()) works unchanged,
 * since both factories hand back a real {@code Follower}.
 */
@Autonomous(name = "Pedro Auto Fast 15 (Simulated)", group = "Simulator")
public class PedroAutoFast15Simulated extends CommandOpMode {
    private MecanumDriveSubsystemSimulation driveSim;

    private IntakeSubsystemSimulate intake;

    private ShooterSubsystemSimulate shooter;
    private Follower follower;

    private Fast15 f15;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private final long pickupTime_ms = 2000;
    private final long scoreTime_ms = 3000;

    private int seqNum;


    @Override
    public void initialize() {
        super.reset();


        driveSim = new MecanumDriveSubsystemSimulation(this);
        intake = new IntakeSubsystemSimulate();
        shooter = new ShooterSubsystemSimulate();
        GlobalData.selectStartingConditions(this);


        f15 = new Fast15();

        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/PathChain/FollowPathCommand usage.
        follower = SimulatorConstants.createSimulatedFollower(driveSim);

        f15.buildPaths(follower);

        follower.setStartingPose(f15.start);


        schedule(
                Commands.sequence(

                        new FollowPathCommand(follower, f15.scoreP),
                        shootCommand(),

                        intakeCommand(f15.intake1P),

                        new FollowPathCommand(follower, f15.score1P),
                        shootCommand(),

                        intakeCommand(f15.intake2P),

                        new FollowPathCommand(follower, f15.score2P),
                        shootCommand(),

                        intakeCommand(f15.intake3P),

                        new FollowPathCommand(follower, f15.score3P),
                        shootCommand()));
    }

    public Command shootCommand() {
        return Commands.sequence(
                shooter.runShooterCommand(),
                new WaitCommand(scoreTime_ms),
                shooter.stopShooterCommand());
    }

    public Command intakeCommand(PathChain pc) {
        return
                Commands.sequence(

                        intake.runIntakeCommand(),
                        new FollowPathCommand(follower, pc),

                        new WaitCommand(pickupTime_ms),
                        intake.stopIntakeCommand());
    }


    @Override
    public void run() {
        super.run();
        follower.update();
        telemetryM.addData("Index", seqNum);
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.addData("Busy", follower.isBusy());
        telemetryM.update(telemetry);
    }

    //        @Override
//        public void stop() {
//            r.saveEnd();
//            reset();
//        }

}