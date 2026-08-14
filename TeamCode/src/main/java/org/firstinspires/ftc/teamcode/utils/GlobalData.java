package org.firstinspires.ftc.teamcode.utils;


import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class GlobalData {

    private static boolean redAlliance = false;

    private static boolean blueAlliance = false;

    public static boolean isBlueAlliance() {
        return blueAlliance;
    }

    public static void setBlueAlliance() {
        blueAlliance = true;
        redAlliance = false;
        robotLook = blueLook;
    }

    public static void resetAlliances() {
        blueAlliance = false;
        redAlliance = false;
        robotLook = new Style("", "#3F51B5", 1.0);
        ;
    }

    public static Command setRedAllianceCommand() {
        return new InstantCommand(GlobalData::setRedAlliance);
    }

    public static boolean isRedAlliance() {
        return redAlliance;
    }

    public static void setRedAlliance() {
        redAlliance = true;
        blueAlliance = false;
        robotLook = redLook;
    }

    public static Command setBlueAllianceCommand() {
        return new InstantCommand(GlobalData::setBlueAlliance);
    }

    public static Command toggleAllianceCommand() {
        return new ConditionalCommand(setBlueAllianceCommand(), setRedAllianceCommand(), GlobalData::isRedAlliance);
    }

    public static Style robotLook = new Style("", "#3F51B5", 1.0);
    private static final Style redLook = new Style("", "#FF0000", 1.0);
    private static final Style blueLook = new Style("", "#0000FF", 1.0);

    public static boolean allianceSelected;
    private static boolean choicesComplete;

    private static boolean allianceIsSelected;

    private boolean isAllianceSelected;

    //Method to select starting position and number of artifact groupw using gamepad
    public static void selectStartingConditions(OpMode opMode) {
        opMode.telemetry.setAutoClear(true);
        opMode.telemetry.clearAll();
        allianceSelected = false;
        resetAlliances();
        choicesComplete = false;
        allianceIsSelected = false;

        while (!choicesComplete) {
            opMode.telemetry.addData("Initializing Autonomous for Team:",
                    "Fondy Fire Tech", " ", "22526");
            opMode.telemetry.addData("---------------------------------------", "");
            opMode.telemetry.addData("Select Alliance using Bumpers on gamepad 1:", "");
            opMode.telemetry.addData("    Blue   ", "Left");
            opMode.telemetry.addData("    Red    ", "Right");
            opMode.telemetry.addData("Do Not Press Start Unless Alliance Selection Made", "");
            opMode.telemetry.addLine();

            if (opMode.gamepad1.left_bumper) {
                GlobalData.setBlueAlliance();
                allianceIsSelected = true;
            }
            if (opMode.gamepad1.right_bumper) {
                GlobalData.setRedAlliance();
                allianceIsSelected = true;
            }

            allianceSelected = allianceIsSelected && (GlobalData.isRedAlliance() || GlobalData.isBlueAlliance());

            choicesComplete = allianceSelected;

            if (!allianceIsSelected)
                opMode.telemetry.addData("Alliance ", "not Chosen");
            if (allianceSelected && GlobalData.isRedAlliance())
                opMode.telemetry.addData("RED Alliance Selected", "");
            if (allianceSelected && GlobalData.isBlueAlliance())
                opMode.telemetry.addData("BLUE Alliance Selected", "");
            if (allianceSelected)
                opMode.telemetry.addData("AllianceSelected", allianceSelected);

            opMode.telemetry.update();
            // sleep(50);
        }

        opMode.telemetry.clearAll();
        if (GlobalData.isRedAlliance())
            opMode.telemetry.addData("Alliance Selection ", "RED");
        if (GlobalData.isBlueAlliance())
            opMode.telemetry.addData("Alliance Selection ", "BLUE");
        opMode.telemetry.addLine();

        opMode.telemetry.addLine();
        opMode.telemetry.addData("Restart OpMode ", "to Change");

        opMode.telemetry.update();

    }

}
