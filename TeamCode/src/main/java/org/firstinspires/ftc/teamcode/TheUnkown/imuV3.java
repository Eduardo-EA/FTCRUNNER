package org.firstinspires.ftc.teamcode.TheUnkown;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "imuV3")
public class imuV3 extends OpMode {

    public static final double kP = 0.003;
    public static final double kI = 0.12;
    public static final double kD = 0.053;
    public static final double kThreshold = 8;

    private MecanumDrivingSample mecanumDrivingSample;
    private GamepadEx driverGamepad;

    private PIDLiftController liftController;
    private SimpleLinearLift lift;
    private MotorEx liftMotor;

    Servo RightServo;
    Servo LeftServo;

    BNO055IMU BNO055;
    Orientation angles;

    public static PIDController pid = new PIDController(
            kP, kI, kD
    );


    @Override
    public void init() {
        driverGamepad = new GamepadEx(gamepad1);


        pid.setTolerance(kThreshold);
        pid.reset();

        liftMotor = new MotorEx(hardwareMap, "LiftMotor");
        lift = new SimpleLinearLift(liftMotor);
        liftController = new PIDLiftController(lift);
    }

    @Override
    public void loop() {
        liftController.power(gamepad2.left_stick_y);

        if (gamepad2.a) {
            liftController.setStageOne();
        } else if (gamepad2.b) {
            liftController.setStageTwo();
        } else if (gamepad2.x) {
            liftController.setStageThree();
        } else if (gamepad2.y) {
            liftController.resetStage();
        }

    }
}
