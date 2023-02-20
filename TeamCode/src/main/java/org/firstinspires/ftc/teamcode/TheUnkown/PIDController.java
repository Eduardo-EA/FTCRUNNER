package org.firstinspires.ftc.teamcode.TheUnkown;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous
public class PIDController {
    private double kp; // proportional gain
    private double ki; // integral gain
    private double kd; // derivative gain

    private double target; // target value
    private double error; // error between target and current value
    private double integral; // sum of error over time
    private double previousError; // error from previous iteration

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

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
}
