package org.firstinspires.ftc.teamcode.WorkInProgress;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
@Config
public class PIDTEST extends LinearOpMode {

    DcMotor motor;

    public static double kp = 0.5; //proportional gain
    public static double ki = 0.01; //integral gain
    public static double kd = 0.1; //derivative gain

    //define variables
    double error = 0; //difference between desired value and actual value
    double prev_error = 0; //previous error
    double integral = 0; //sum of all errors
    double derivative = 0; //change in error over time




    double high = (2 * COUNTS_PER_ROTATION);


    private static final double COUNTS_PER_ROTATION = 751.8;
    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class,"LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //define constants

        waitForStart();
//loop
        while(opModeIsActive()) {
            //get sensor values
            int position = motor.getCurrentPosition();

            //calculate error
            error = high - position;

            //calculate integral and derivative
            integral += error;
            derivative = error - prev_error;
            prev_error = error;

            //calculate motor power
            double power = kp * error + ki * integral + kd * derivative;

            //set motor power
           // motor.setPower(power);

            //wait for next loop
           // wait(10);

            if (gamepad1.a) {
                double targetPosition = COUNTS_PER_ROTATION * 3.5;
                motor.setTargetPosition((int) targetPosition);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(power);
            }
            while (motor.isBusy()) {
                telemetry.addData("Current Position", motor.getCurrentPosition());
                telemetry.addData("Target Position", motor.getTargetPosition());
                telemetry.update();
            }
        }

        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Target Position", motor.getTargetPosition());
        telemetry.update();

    }
}
