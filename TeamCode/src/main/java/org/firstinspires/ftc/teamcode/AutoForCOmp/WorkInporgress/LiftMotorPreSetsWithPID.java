package org.firstinspires.ftc.teamcode.AutoForCOmp.WorkInporgress;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TheUnkown.PIDController;


@TeleOp(name="PIDHELPME", group="TeleOp")
public class LiftMotorPreSetsWithPID extends OpMode {
    private double kp; // proportional gain
    private double ki; // integral gain
    private double kd; // derivative gain

    private double target; // target value
    private double error; // error between target and current value
    private double integral; // sum of error over time
    private double previousError; // error from previous iteration
    private DcMotor motor;

    private static final double COUNTS_PER_ROTATION = 751.8;

    PIDController pidController = new PIDController(0.1, 0.05, 0.02);


    public double calculate(double currentValue) {
        error = target - currentValue;
        integral += error;
        double derivative = error - previousError;
        previousError = error;
        return kp * error + ki * integral + kd * derivative;
    }
    public void setTarget(double target) {
        this.target = target;
    }

    double High = 10;
    double middle = 5;
    double low = 3;

    double Highjunction = (High * COUNTS_PER_ROTATION);
    double Middlejunction = (middle * COUNTS_PER_ROTATION);
    double Lowjunction = (low * COUNTS_PER_ROTATION);

    ElapsedTime timer = new ElapsedTime();



    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


// apply the output value to the motor

    @Override
    public void loop() {



      if (gamepad1.a) {
          pidController.setTarget(Highjunction);
          double output = pidController.calculate(motor.getCurrentPosition());
          motor.setTargetPosition((int) output);
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
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


}
