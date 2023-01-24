package org.firstinspires.ftc.teamcode.TheUnkown;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDFORDRIVE extends LinearOpMode {

    DcMotorEx LiftMotor;

    double kp = 0;
    double ki = 0;
    double kd =0;

    double reference = 0;
    double lastReference = reference;
    double intergralsum = 0;

    double lastError = 0;

    double maxIntegralSum = 0;

    double a = 0.8; // a can be from anything from 0 < a < 1

    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;




    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        LiftMotor = hardwareMap.get(DcMotorEx.class,"LiftMotor");
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

    }

    public double PIDControl() {
        double encoderPosition = LiftMotor.getCurrentPosition();
        double error = reference - encoderPosition;

        double errorChange = (error - lastError);

        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate / timer.seconds();
        intergralsum = intergralsum + (error * timer.seconds());

        if (intergralsum > maxIntegralSum) {
            intergralsum = maxIntegralSum;
        }
        if (intergralsum < -maxIntegralSum) {
            intergralsum = -maxIntegralSum;
        }
        if (reference != lastReference) {
            intergralsum = 0;
        }

        double out = (kp * error) + (ki * intergralsum) + (kd * derivative);

        LiftMotor.setPower(out);

        lastError = error;
        lastReference = reference;

        timer.reset();





        timer.reset();

        double output = (error * kp) + (derivative * kd) + (intergralsum * ki);
        return output;
    }
}
