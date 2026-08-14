package org.firstinspires.ftc.teamcode.utils;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Fast15 {

    public Pose start = new Pose(24 + 6.25, 120 + 8 + 4.75, Math.toRadians(90));
    public Pose scorePControl = new Pose(55.593, 94.779);
    public Pose score = new Pose(48, 96.0, Math.toRadians(135)); // score
    public Pose intake1 = new Pose(17, 83.5, Math.toRadians(180)); // intake\
    public Pose intake1Control = new Pose(50.000, 87.5);
    public Pose intake2 = new Pose(10, 60.050, Math.toRadians(-170)); // intake
    public Pose intake2Control = new Pose(65.400, 65);
    public Pose gate = new Pose(16.25, 72.500, Math.toRadians(180)); //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
    public Pose gateControl = new Pose(30, 73); //62);
    public Pose intake3 = new Pose(10, 39.750 - 3.5, Math.toRadians(180));
    public Pose intake3Control = new Pose(75, intake3.getY() - 5);


    public Pose intakeCorner = new Pose(6.5, 10, Math.toRadians(270));
    public Pose intakeCornerControl = intakeCorner.withY(50);
    public Pose scoreToCorner = score.withHeading(Math.toRadians(-135));
    public Pose scoreCorner = score; //new Pose(56, 20, Math.toRadians(180));

    public Pose score3Control = new Pose(38, 68);
    public Pose park = new Pose(48, 72, Math.toRadians(180));//new Pose(36, 12, Math.toRadians(180));

    public int getIndex() {
        return index;
    }

    private int index;

    public PathChain scoreP, score1P, score2P, score3P, scoreCornerP, intake1P, intake2P, intake3P, intakeCornerP, gateP;

    public Fast15() {


        if (GlobalData.isRedAlliance()) {
            start = start.mirror();
            scorePControl = scorePControl.mirror();
            score = score.mirror();
            intake1 = intake1.mirror();
            intake1Control = intake1Control.mirror();
            gate = gate.mirror();
            gateControl = gateControl.mirror();
            intake2 = intake2.mirror();
            intake2Control = intake2Control.mirror();
            intake3 = intake3.mirror();
            intake3Control = intake3Control.mirror();
            intakeCorner = intakeCorner.mirror();
            intakeCornerControl = intakeCornerControl.mirror();
            score3Control = score3Control.mirror();
            scoreToCorner = scoreToCorner.mirror();
            scoreCorner = scoreCorner.mirror();
            park = park.mirror();
        }

        index = 0;
    }


    public void buildPaths(Follower follower) {
        scoreP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                scorePControl,
                                score
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();

        score1P = follower.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();

        score2P = follower.pathBuilder()
                .addPath(new BezierLine(intake2, score))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();

        score3P = follower.pathBuilder()
                .addPath(new BezierCurve(intake3, score3Control, scoreCorner))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake3.getHeading(), scoreCorner.getHeading())
                .build();

        intake1P = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake1Control,
                                intake1
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();

        intake2P = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake2Control,
                                intake2
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading(), 0.5)
                .build();

        intake3P = follower.pathBuilder()
                .addPath(new BezierCurve(score, intake3Control, intake3))
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake3.getHeading(), 0.7)
                .build();

        gateP=follower.pathBuilder()
                .addPath(new BezierCurve(intake2, gateControl, gate))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), gate.getHeading())
                .build();

    }


    public boolean hasNext() {
        int PATH_COUNT = 9;
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}