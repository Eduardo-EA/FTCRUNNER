package org.firstinspires.ftc.teamcode.AutoForCOmp;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="ARMTEST", group="Linear Opmode")
public class Lift extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();  //delete final if prob
    DcMotor L;
    DcMotor R;

    public volatile boolean dpad_down;
    public volatile boolean dpad_up;





    @Override
    public void runOpMode() {


        L = hardwareMap.get(DcMotor.class, "L");
        R = hardwareMap.get(DcMotor.class, "R");





        L.setDirection(DcMotorSimple.Direction.REVERSE);



        R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {




            if (gamepad1.dpad_up) {

                L.setPower(.75);
                R.setPower(.75);
            }
            else if (gamepad1.dpad_down) {
                L.setPower(-1);
                R.setPower(-1);
            } else if (!gamepad1.dpad_down && !gamepad2.dpad_up) {
                L.setPower(0);
                R.setPower(0);
            }

        }
    }

}