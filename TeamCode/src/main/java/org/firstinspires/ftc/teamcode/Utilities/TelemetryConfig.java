package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Central configuration for ThunderBot2026.
 * Use FTC Dashboard to toggle these values live.
 */
@Config
public class TelemetryConfig
{
    // --- GLOBAL TELEMETRY MASTER SWITCHES ---
    public static boolean SHOW_DEBUG_ALL = false;
    public static boolean SHOW_PID_DETAILS = false; // Toggles error/integral terms globally

    // --- SUBSYSTEM SPECIFIC SWITCHES ---
    public static boolean DEBUG_TURRET = false;
    public static boolean DEBUG_LAUNCHER_FACADE = false;
    public static boolean DEBUG_FLYWHEEL = false;
    public static boolean DEBUG_TRANSFER_FACADE = false;
    public static boolean DEBUG_TURNSTILE = false;
    public static boolean DEBUG_DRIVE = false;
    public static boolean DEBUG_INTAKE = false;
    public static boolean DEBUG_LIMELIGHT = false;
    public static boolean DEBUG_BEAMBREAK = false;
    public static boolean DEBUG_BALL_SENSOR  = false;

    // --- HARDWARE DIAGNOSTICS ---
    // Toggle this to see motor currents, voltages, and loop times
    public static boolean SHOW_HARDWARE_DIAGNOSTICS = false;
}