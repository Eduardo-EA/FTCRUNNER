package org.firstinspires.ftc.teamcode.AutoForCOmp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoForCOmp.CamTune.Section1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous
public class LeftCamForAuto extends LinearOpMode
{

    Servo RightServo;
    Servo LeftServo;
    DcMotor LiftMotor;



    static final double COUNTS_PER_MOTOR_REV = 3895.9;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double LiftSpeed = 1;

    private final ElapsedTime runtime = new ElapsedTime();

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode()
    {

        RightServo = hardwareMap.get(Servo.class,"RightServo");
        LeftServo = hardwareMap.get(Servo.class,"LeftServo");

        //Close servos
        RightServo.setPosition(.55);
        LeftServo.setPosition(.48);

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Lift Motor THe speed is 1
        LiftMotor.setPower(LiftSpeed);
        sleep(250);    //Change sleep timer for the duration of the motor lift
        LiftMotor.setPower(0);

        Trajectory Forward_1in = drive.trajectoryBuilder(new Pose2d())
                .forward(1)  //change the values here to make the robot movement increase or decrease.  + for Forward - for reverse
                .build();

        Trajectory StrafeLeft = drive.trajectoryBuilder(Forward_1in.end())
                .strafeRight(5)  //if you want to strafe right you do .strafeRight() you can not just add a negative
                .build();
        Trajectory Forward_6in = drive.trajectoryBuilder(StrafeLeft.end())
                .forward(6.3)
                .build();

        TrajectorySequence turn_2 = drive.trajectorySequenceBuilder(Forward_6in.end())
                .turn(Math.toRadians(1.6)) //change here to change the degree of the turn
                .build();
        TrajectorySequence turn_75 = drive.trajectorySequenceBuilder(turn_2.end())
                .turn(Math.toRadians(-7.5))
                .build();


        Trajectory Forward_1_8 = drive.trajectoryBuilder(turn_75.end())
                .forward(1.8)
                .build();
        Trajectory Backward_1_8 = drive.trajectoryBuilder(Forward_1_8.end())
                .forward(-1.8)
                .build();


        TrajectorySequence Turn90 = drive.trajectorySequenceBuilder(Forward_6in.end())
                .turn(Math.toRadians(-90))
                .build();
        Trajectory swaferight = drive.trajectoryBuilder(Turn90.end())
                .strafeRight(2)
                .build();
        Trajectory swaferight2 = drive.trajectoryBuilder(Turn90.end())
                .strafeRight(4)
                .build();
        Trajectory swafeleft2 = drive.trajectoryBuilder(swaferight2.end())
                .strafeLeft(2)
                .build();
        Trajectory swafeleft3 = drive.trajectoryBuilder(swafeleft2.end())
                .strafeLeft(4)
                .build();
        //If you want to add more movement copy and paste the code above line 120 to 122


        if (isStopRequested()) return;



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

//rep wait for start
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }



        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        //left 1 middle 2 Right 3
        /* Actually do something useful */
        if(tagOfInterest.id == LEFT){ //1
            //trajectory


            drive.followTrajectory(Forward_1in);
            drive.followTrajectory(StrafeLeft);
            drive.followTrajectory(Forward_6in);
            drive.followTrajectorySequence(turn_2);

            LiftMotor.setPower(LiftSpeed);
            sleep(2400);
            LiftMotor.setPower(0);

            drive.followTrajectorySequence(turn_75);
            drive.followTrajectory(Forward_1_8);
            drive.followTrajectory(Backward_1_8);

            LiftMotor.setPower(-1);
            sleep(2400);
            LiftMotor.setPower(0);









        }else if(tagOfInterest.id == MIDDLE){ //2

            drive.followTrajectory(Forward_1in);
            drive.followTrajectory(StrafeLeft);
            drive.followTrajectory(Forward_6in);
            drive.followTrajectorySequence(turn_2);

            drive.followTrajectory(swafeleft2);



        }else if (tagOfInterest.id == RIGHT){ //3

            drive.followTrajectory(Forward_1in);
            drive.followTrajectory(StrafeLeft);
            drive.followTrajectory(Forward_6in);
            drive.followTrajectorySequence(turn_2);



            drive.followTrajectory(swafeleft3);







        }





    }
/*
    private void encoderDrive(double LiftMotorInches){
        int newLiftMotorTarget;

        if (opModeIsActive()) {
            newLiftMotorTarget = LiftMotor.getCurrentPosition() + (int)(LiftMotorInches * COUNTS_PER_INCH);

            LiftMotor.setTargetPosition(newLiftMotorTarget);
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            LiftMotor.setPower(Math.abs(LiftSpeed));

        }
    }

 */

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}