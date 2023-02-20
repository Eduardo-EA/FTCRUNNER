package org.firstinspires.ftc.teamcode.WorkInProgress;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous
public class AnotherEncoderTEst extends LinearOpMode {
    private DcMotor motor;
    private static final double COUNTS_PER_ROTATION = 751.8;
    private static final double ROTATIONS = 5;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double targetPosition = COUNTS_PER_ROTATION * ROTATIONS;
        motor.setTargetPosition((int) targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", motor.getTargetPosition());
            telemetry.update();
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
