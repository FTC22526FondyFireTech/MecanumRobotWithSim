package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Configurables {

    public static boolean redAlliance = true;

    //Telemetry
    public static boolean showPoseTelemetry = false;
    public static boolean showEncoderTelemetry = false;
    public static boolean showMotorTelemetry = false;
    public static boolean showDebugTelemetry = false;
    public static boolean showIntakeTelemetry = false;
    public static boolean showShooterTelemetry = false;

    //Simulation

    public static boolean doSimulation = true;

    public static double intakePowerSim = .5;
    public static double shooterPowerSim = .75;

    public static boolean test = false;


}
