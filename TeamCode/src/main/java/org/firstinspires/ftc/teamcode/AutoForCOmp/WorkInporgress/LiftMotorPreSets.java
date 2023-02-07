package org.firstinspires.ftc.teamcode.AutoForCOmp.WorkInporgress;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name="LiftControlPreSets", group="TeleOp")
public class LiftMotorPreSets extends OpMode {
    private DcMotor motor;
    private int positionA = 0;
    private int positionB = 100;
    private int positionY = 200;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motor.setTargetPosition(positionA);
            motor.setPower(1);
            while (motor.isBusy()) {
                // wait until the motor reaches the target position
            }
            motor.setPower(0);
            telemetry.addData("Motor Position", "A");
        } else if (gamepad1.b) {
            motor.setTargetPosition(positionB);
            motor.setPower(1);
            while (motor.isBusy()) {
                // wait until the motor reaches the target position
            }
            motor.setPower(0);
            telemetry.addData("Motor Position", "B");
        } else if (gamepad1.y) {
            motor.setTargetPosition(positionY);
            motor.setPower(1);
            while (motor.isBusy()) {
                // wait until the motor reaches the target position
            }
            motor.setPower(0);
            telemetry.addData("Motor Position", "Y");
        } else {
            telemetry.addData("Motor Position", "None");
        }
        telemetry.update();
    }
}
