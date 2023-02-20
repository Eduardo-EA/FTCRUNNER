package org.firstinspires.ftc.teamcode.AutoForCOmp.WorkInporgress;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LiftControlPreSets", group="TeleOp")
public class LiftMotorPreSets extends OpMode {
    private DcMotor motor;
    private static final double COUNTS_PER_ROTATION = 751.8;


    double High = 10;
    double middle = 5;
    double low = 3;



    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
      if (gamepad1.a) {
          double targetPosition = COUNTS_PER_ROTATION * High;
          motor.setTargetPosition((int) targetPosition);
          motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motor.setPower(1);
      } else if (gamepad1.b){
          double targetPosition = COUNTS_PER_ROTATION * middle;
          motor.setTargetPosition((int) targetPosition);
          motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motor.setPower(1);
      }else if (gamepad1.y){
          double targetPosition = COUNTS_PER_ROTATION * low;
          motor.setTargetPosition((int) targetPosition);
          motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motor.setPower(1);
      } else if (gamepad1.x){
          motor.setTargetPosition(0);
          motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motor.setPower(1);
      }else {
          motor.setPower(0);
          motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }

        while (motor.isBusy()) {
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", motor.getTargetPosition());
            telemetry.update();
        }
       // motor.setPower(0);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad1.right_bumper){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1);
        } else if (gamepad1.left_bumper) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-1);
        }
    }


}
