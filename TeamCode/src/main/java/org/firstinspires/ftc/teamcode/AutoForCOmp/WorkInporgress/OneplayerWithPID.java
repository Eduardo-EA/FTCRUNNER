package org.firstinspires.ftc.teamcode.AutoForCOmp.WorkInporgress;


import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="oneplayerWithPID", group="Linear Opmode")
public class OneplayerWithPID extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();  //delete final if prob
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightBackDrive;
    DcMotor LeftBackDrive;

    DcMotor LiftMotor;

    Servo RightServo;
    Servo LeftServo;

    BNO055IMU BNO055;
    Orientation angles;

    boolean oldServoButton;
    boolean oldcontrol;

    public volatile boolean dpad_down;
    public volatile boolean dpad_up;

    public static double kp = 0.5; //proportional gain
    public static double ki = 0.01; //integral gain
    public static double kd = 0.1; //derivative gain

    //define variables
    double error = 0; //difference between desired value and actual value
    double prev_error = 0; //previous error
    double integral = 0; //sum of all errors
    double derivative = 0; //change in error over time
    double high = (3.5 * COUNTS_PER_ROTATION);

    private static final double COUNTS_PER_ROTATION = 394.5;





    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNO055 = hardwareMap.get(BNO055IMU.class, "IMU");
        BNO055.initialize(parameters);

        RightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        RightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        LeftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");


        RightServo = hardwareMap.get(Servo.class, "RightServo");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");

        RightServo.setPosition(.35);
        LeftServo.setPosition(.65);

        oldServoButton = false;
        oldcontrol = false;


        int position = LiftMotor.getCurrentPosition();
        error = high - position;
        integral += error;
        derivative = error - prev_error;
        prev_error = error;


        double power = kp * error + ki * integral + kd * derivative;

        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            angles = BNO055.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float pi = (float) 3.14159;
            float gyroRadians = angles.firstAngle * pi / 180;

            float axial2 = (float) (axial * cos(gyroRadians) - lateral * sin(gyroRadians));
            lateral = axial * sin(gyroRadians) + lateral * cos(gyroRadians);
            axial = axial2;

            double leftFrontSign  = Math.signum((axial + lateral + yaw));
            double rightFrontSign = Math.signum((axial - lateral - yaw));
            double leftBackSign   = Math.signum((axial - lateral + yaw));
            double rightBackSign  = Math.signum((axial + lateral - yaw));

            double leftFrontPower  = /*leftFrontSign */ pow((axial + lateral + yaw), 3);
            double rightFrontPower = /*rightFrontSign */ pow((axial - lateral - yaw), 3);
            double leftBackPower   = /*leftBackSign */ pow((axial - lateral + yaw), 3);
            double rightBackPower  = /*rightBackSign */ pow((axial + lateral - yaw), 3);


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            LeftFrontDrive.setPower(leftFrontPower * 3/4);
            RightFrontDrive.setPower(rightFrontPower * 3/4);
            LeftBackDrive.setPower(leftBackPower* 3/4);
            RightBackDrive.setPower(rightBackPower * 3/4 );




            //grab cone
            if (gamepad1.right_bumper) {
                RightServo.setPosition(.35);
                LeftServo.setPosition(.65);
                //let go of cone
            } else if (gamepad1.left_bumper){
                RightServo.setPosition(.55);
                LeftServo.setPosition(.48);
            }



                //This one uses a boolean for a varible if true runs with encoders but if false runs without
            double currentpos = LiftMotor.getCurrentPosition();
            boolean ispresent = false;

            if (gamepad1.a){
                ispresent = true;
            }else if (gamepad1.b){
                ispresent = true;
            }else if (gamepad1.y){
                ispresent = true;
            }else if (gamepad1.dpad_up){
                ispresent = false;
            } else if (gamepad1.dpad_down){
                ispresent = false;
            }


            if (gamepad1.a) {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition((int) (COUNTS_PER_ROTATION * 3.8));
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(power);
            } else if (gamepad1.b) {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition((int) (COUNTS_PER_ROTATION * 2));
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(power);
            } else if (gamepad1.y) {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition((int) (COUNTS_PER_ROTATION * 1));
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(power);
            }

            if (gamepad1.dpad_up){
                LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LiftMotor.setPower(1);
            } else if  (gamepad1.dpad_down) {
                LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LiftMotor.setPower(-1);
            } else if (!ispresent) {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition((int) currentpos);
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(power);
            }

/*                  Another Test But this one subtracts or add 2 front the current position
            double lifttestright = LiftMotor.getCurrentPosition() +2;
            double lifttestleft = LiftMotor.getCurrentPosition() -2;

            if (gamepad1.right_trigger > 0) {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition((int) lifttestright);
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(power);
            } else if (gamepad1.left_trigger > 0){
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition((int) lifttestleft);
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(power);
            }

 */
                //add telemetry for the liftmotor

            telemetry.addData("Ticks","This is FUN: " + LiftMotor.getCurrentPosition());

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("YAW", angles.firstAngle);
            telemetry.update();


        }
    }

}