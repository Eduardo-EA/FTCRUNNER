package org.firstinspires.ftc.teamcode.AutoForCOmp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Config
@Autonomous
public class LeftscoreV2 extends LinearOpMode
{

    Servo RightServo;
    Servo LeftServo;
    DcMotor LiftMotor;

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
    Pose2d myPose = new Pose2d(0, 0, Math.toRadians(90));


    @Override
    public void runOpMode()
    {

        RightServo = hardwareMap.get(Servo.class,"RightServo");
        LeftServo = hardwareMap.get(Servo.class,"LeftServo");

        //Close servos
        RightServo.setPosition(.35);
        LeftServo.setPosition(.65);


        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Lift Motor THe speed is 1
        LiftMotor.setPower(LiftSpeed);
        sleep(1000);    //Change sleep timer for the duration of the motor lift
        LiftMotor.setPower(0);

     //   LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        drive.setPoseEstimate(myPose);
        //Park NUmber 2

        TrajectorySequence Moveforward1 = drive.trajectorySequenceBuilder(myPose)
                .lineToConstantHeading(new Vector2d(0,7.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  //add temp markers if it doesnt work
                        LiftMotor.setPower(LiftSpeed))
                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                        LiftMotor.setPower(0))
                .build();
        TrajectorySequence TurnToScore = drive.trajectorySequenceBuilder(Moveforward1.end())
                .turn(Math.toRadians(-45)) // Turns 45 degrees counter-clockwise If it doesnt work do -6.5
                .build();
        Trajectory  ForwardToScore = drive.trajectoryBuilder(TurnToScore.end())
                .lineToConstantHeading(new Vector2d(1.1,8.6))
                .build();
        Trajectory ForwardToScoreFineTune = drive.trajectoryBuilder(ForwardToScore.end())
                .lineToConstantHeading(new Vector2d(2.2,9.8))
                .addDisplacementMarker(() -> {
                    RightServo.setPosition(.55);
                    LeftServo.setPosition(.48);
                })
                .build();
        Trajectory BackToScore = drive.trajectoryBuilder(ForwardToScoreFineTune.end())
                .lineToConstantHeading(new Vector2d(1.1,8.6))
                .build();
        TrajectorySequence StraightenOut = drive.trajectorySequenceBuilder(BackToScore.end())
                .turn(Math.toRadians(45)) // 7 if problem
                .UNSTABLE_addTemporalMarkerOffset(0, () ->
                        LiftMotor.setPower(-LiftSpeed))
                .UNSTABLE_addTemporalMarkerOffset(1, () ->
                        LiftMotor.setPower(0))
                .build();





        Trajectory one = drive.trajectoryBuilder(new Pose2d())
                .forward(.5)
                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .strafeLeft(7)
                .build();

        Trajectory three = drive.trajectoryBuilder(one.end())
                .strafeRight(6.3)
                .build();

        Trajectory four = drive.trajectoryBuilder(one.end())
                .forward(4)
                .build();
        Trajectory five = drive.trajectoryBuilder(four.end())
                .forward(1.5)
                .addDisplacementMarker(() ->{
        LiftMotor.setPower(LiftSpeed);
        sleep(1900);
        LiftMotor.setPower(0);

        LiftMotor.setPower(-.4);
        sleep(500);
        LiftMotor.setPower(0);
    })
                .build();

        TrajectorySequence turn = drive.trajectorySequenceBuilder(four.end())
                .turn(Math.toRadians(-8.8))
                .build();
        Trajectory six = drive.trajectoryBuilder(turn.end())
                .forward(.09)
                .addDisplacementMarker(()->{
                    sleep(500);
                    RightServo.setPosition(.55);
                    LeftServo.setPosition(.48);
                })
                .build();
        Trajectory seven = drive.trajectoryBuilder(one.end())
                        .forward(7)
                                .build();
        TrajectorySequence turn2 = drive.trajectorySequenceBuilder(four.end())
                .turn(Math.toRadians(-6.5))
                .build();
        Trajectory eight = drive.trajectoryBuilder(one.end())
                .strafeLeft(6)
                .build();
        Trajectory nine = drive.trajectoryBuilder(one.end())
                .forward(-1.7)
                .build();
        TrajectorySequence turn3 = drive.trajectorySequenceBuilder(four.end())
                .turn(Math.toRadians(8))
                .build();
        Trajectory ten = drive.trajectoryBuilder(six.end())
                .forward(-2.2)
                .build();
        TrajectorySequence turn4 = drive.trajectorySequenceBuilder(four.end())
                .turn(Math.toRadians(7))
                .build();
        Trajectory strafe = drive.trajectoryBuilder(turn3.end())
                .strafeRight(7)
                .build();




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
       /* if(tagOfInterest.id == LEFT){ //1
            //trajectory
            drive.followTrajectory(one);
            drive.followTrajectory(seven);
            drive.followTrajectorySequence(turn2);
            drive.followTrajectory(five);
            // LiftMotor.setPower(-.3);
            // sleep(300);
            //  LiftMotor.setPower(0);
            drive.followTrajectory(six);
            drive.followTrajectory(nine);
            drive.followTrajectorySequence(turn3);
            drive.followTrajectory(eight);
            drive.turn(Math.toRadians(4));



        }else if(tagOfInterest.id == MIDDLE){ //2
            drive.followTrajectory(one);
            drive.followTrajectory(seven);
            drive.followTrajectorySequence(turn2);
            drive.followTrajectory(five);
            // LiftMotor.setPower(-.3);
            // sleep(300);
            //  LiftMotor.setPower(0);
            drive.followTrajectory(six);



        }else{ //3
            drive.followTrajectory(one);
            drive.followTrajectory(three);
            drive.followTrajectory(four);
            drive.followTrajectorySequence(turn);
            drive.followTrajectory(five);
           // LiftMotor.setPower(-.3);
           // sleep(300);
          //  LiftMotor.setPower(0);
            drive.followTrajectory(six);
            drive.followTrajectory(ten);


        }

        */


        if(tagOfInterest == null){
            drive.followTrajectory(one);
            drive.followTrajectory(seven);
            drive.followTrajectorySequence(turn2);
            drive.followTrajectory(five);
            drive.followTrajectory(six);
            drive.followTrajectory(ten);
            drive.followTrajectorySequence(turn4);
        }else{
            switch(tagOfInterest.id){
                case 1:
                    drive.followTrajectory(one);
                    drive.followTrajectory(seven);
                    drive.followTrajectorySequence(turn2);
                    drive.followTrajectory(five);

                    drive.followTrajectory(six);
                    drive.followTrajectory(nine);
                    drive.followTrajectorySequence(turn3);
                    drive.followTrajectory(eight);


                    break;

                case 2:
                   drive.followTrajectorySequence(Moveforward1);
                   drive.followTrajectorySequence(TurnToScore);
                   drive.followTrajectory(ForwardToScore);
                   drive.followTrajectory(ForwardToScoreFineTune);
                   drive.followTrajectory(BackToScore);
                   drive.followTrajectorySequence(StraightenOut);
                    break;

                case 3:
                    drive.followTrajectory(one);
                    drive.followTrajectory(seven);
                    drive.followTrajectorySequence(turn2);
                    drive.followTrajectory(five);

                    drive.followTrajectory(six);
                    drive.followTrajectory(nine);
                   // drive.followTrajectorySequence(turn3);
                    drive.turn(Math.toRadians(6));
                    drive.followTrajectory(strafe);
                   // drive.followTrajectory(eight);
                   // drive.followTrajectory(turn);

                    break;
            }
        }


    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
       /* telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

        */
    }
}