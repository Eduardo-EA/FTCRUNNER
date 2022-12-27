package org.firstinspires.ftc.teamcode.AutoForCOmp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RightVersion2 extends LinearOpMode
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
/*
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

 */

    // We define the current state we're on
    // Default to IDLE
   // State currentState = State.IDLE;
    // Define our start pose
    Pose2d startPose = new Pose2d(37.8,-62, Math.toRadians(90));


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

        drive.setPoseEstimate(startPose);

        LiftMotor.setPower(LiftSpeed);
        sleep(250);    //Change sleep timer for the duration of the motor lift
        LiftMotor.setPower(0);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(37.8,-61))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(12.8,-62))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .splineToLinearHeading(new Pose2d(13,-61, Math.toRadians(90)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(12.8,-10, Math.toRadians(45)), Math.toRadians(90))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineTo(new Vector2d(14.8,-8))
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(12.8,-12, Math.toRadians(0)))
                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .lineToLinearHeading(new Pose2d(56,-12, Math.toRadians(0)))
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .lineToLinearHeading(new Pose2d(35,-12, Math.toRadians(135)))
                .build();
        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .lineToLinearHeading(new Pose2d(32,-8, Math.toRadians(135)))
                .build();
        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .lineToLinearHeading(new Pose2d(35,-12, Math.toRadians(0)))
                .build();
        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .lineToLinearHeading(new Pose2d(56,-12, Math.toRadians(0)))
                .build();

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
            drive.followTrajectory(trajectory1);
            drive.followTrajectory(trajectory2);
            drive.followTrajectory(trajectory3);
            drive.followTrajectory(trajectory4);
            drive.followTrajectory(trajectory5);
            drive.followTrajectory(trajectory6);
            drive.followTrajectory(trajectory7);
            drive.followTrajectory(trajectory8);
            drive.followTrajectory(trajectory9);
            drive.followTrajectory(trajectory10);


        }else if(tagOfInterest.id == MIDDLE){ //2
            drive.followTrajectory(trajectory1);
            drive.followTrajectory(trajectory2);
            drive.followTrajectory(trajectory3);
            drive.followTrajectory(trajectory4);
            drive.followTrajectory(trajectory5);
            drive.followTrajectory(trajectory6);
            drive.followTrajectory(trajectory7);
            drive.followTrajectory(trajectory8);
            drive.followTrajectory(trajectory9);
            drive.followTrajectory(trajectory10);



        }else{ //3
            drive.followTrajectory(trajectory1);
            drive.followTrajectory(trajectory2);
            drive.followTrajectory(trajectory3);
            drive.followTrajectory(trajectory4);
            drive.followTrajectory(trajectory5);
            drive.followTrajectory(trajectory6);
            drive.followTrajectory(trajectory7);
            drive.followTrajectory(trajectory8);
            drive.followTrajectory(trajectory9);
            drive.followTrajectory(trajectory10);


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