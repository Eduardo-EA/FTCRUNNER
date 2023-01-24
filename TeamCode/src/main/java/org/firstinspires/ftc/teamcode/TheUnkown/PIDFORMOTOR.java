package org.firstinspires.ftc.teamcode.TheUnkown;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDFORMOTOR extends LinearOpMode {

    DcMotorEx LiftMotor;

    double intergralsum = 0; // try 8
    double kp = 0.003;
    double ki = 0.12;
    double kd = 0.053;

    //

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        LiftMotor = hardwareMap.get(DcMotorEx.class,"LiftMotor");
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
           double power = PIDControl(100, LiftMotor.getCurrentPosition());
           LiftMotor.setPower(power);

        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        intergralsum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;



        timer.reset();

        double output = (error * kp) + (derivative * kd) + (intergralsum * ki);
        return output;
    }
}
