 package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {


    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;


    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(15, .5, 12,   //Increase the P value first while the I and D is at zero
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = (1 * 0.96443093); // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13.04 ; // in


    public static double kV = 1 / rpmToVelocity(MAX_RPM);
    public static double kA = 0 ;
    public static double kStatic = 0 ;


    public static double MAX_VEL =  64.85423764473487;
    public static double MAX_ACCEL = 64.85423764473487;
    public static double MAX_ANG_VEL = Math.toRadians(274.918);
    public static double MAX_ANG_ACCEL = Math.toRadians(275.3762447525618);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
         return 32767 / ticksPerSecond;
    }
}
