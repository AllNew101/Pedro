package org.firstinspires.ftc.teamcode.opmode.auto;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;


@Autonomous(name = "S6")
public class S6 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int mec;
    private double adjust_x;
    private double adjust_y;
    private Servo rightservo;
    private Servo leftservo;
    private Servo neep;
    private Servo spin;
    private Servo wrist;
    private Servo P3;

    private DcMotor L2 ;
    private DcMotor L1 ;
    private long time2;
    private boolean check = false;
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    ColorBlobLocatorProcessor.Blob myBlob;
    RotatedRect myBoxFit;
    org.opencv.core.Rect mySize;
    org.opencv.core.Point[] myPoints;
    List data_right;


    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose plan_1 = new Pose(0,1.5,Math.toRadians(0));
    private final Pose plan_2 = new Pose(0,3,Math.toRadians(0));
    private final Pose plan_3 = new Pose(0,4.5,Math.toRadians(0));
    private final Pose plan_4 = new Pose(0,6,Math.toRadians(0));
    private final Pose plan_5 = new Pose(0,7.5,Math.toRadians(0));
    private final Pose plan1_1 = new Pose(2,1.5,Math.toRadians(0));
    private final Pose plan2_1 = new Pose(2,3,Math.toRadians(0));
    private final Pose plan3_1 = new Pose(2,4.5,Math.toRadians(0));
    private final Pose plan4_1 = new Pose(2,6,Math.toRadians(0));
    private final Pose plan5_1 = new Pose(2,7.5,Math.toRadians(0));
    private final Pose plan1_2 = new Pose(4,1.5,Math.toRadians(0));
    private final Pose plan2_2 = new Pose(4,3,Math.toRadians(0));
    private final Pose plan3_2 = new Pose(4,4.5,Math.toRadians(0));
    private final Pose plan4_2 = new Pose(4,6,Math.toRadians(0));
    private final Pose plan5_2 = new Pose(4,7.5,Math.toRadians(0));
    private final Pose plan1_3 = new Pose(-2,1.5,Math.toRadians(0));
    private final Pose plan2_3 = new Pose(-2,3,Math.toRadians(0));
    private final Pose plan3_3 = new Pose(-2,4.5,Math.toRadians(0));
    private final Pose plan4_3 = new Pose(-2,6,Math.toRadians(0));
    private final Pose plan5_3 = new Pose(-2,7.5,Math.toRadians(0));





    private Path Point1_set;
    private PathChain keep50, keep51, keep52,  keep53, keep54,Point4_set,finale,Point9_Sp4Sl,Point10_Sp4keep,hangSp2,keepSp3,hangSp3,hangSp4,hangSp5,keepSp1,keepSp2,Finish,keep5;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        keep50 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(plan_1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), plan_1.getHeading())
                .build();
        keep51 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan_1), new Point(plan_2)))
                .setLinearHeadingInterpolation(plan_1.getHeading(), plan_2.getHeading())
                .build();
        keep52 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan_2), new Point(plan_3)))
                .setLinearHeadingInterpolation(plan_2.getHeading(), plan_3.getHeading())
                .build();
        keep53 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan_3), new Point(plan_4)))
                .setLinearHeadingInterpolation(plan_3.getHeading(), plan_4.getHeading())
                .build();
        keep54 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan_4), new Point(plan_5)))
                .setLinearHeadingInterpolation(plan_4.getHeading(), plan_5.getHeading())
                .build();



    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:

                follower.setMaxPower(1);

                Servo_kan(0.2);
                wrist.setPosition(1);
                neep.setPosition(0);
                track();

                setPathState(1);
                break;
            case 1:
                if(check == true){
                    setPathState(-1);
                }

                else if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    Servo_kan(0.2);
                    wrist.setPosition(1);
                    neep.setPosition(0);
                    track();

                    follower.followPath(keep50);
                    setPathState(2);


                }
                break;
            case 2:
                if(check == true){
                    setPathState(-1);
                }
                else if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    Servo_kan(0.2);
                    wrist.setPosition(1);
                    neep.setPosition(0);
                    track();

                    follower.followPath(keep51);
                    setPathState(3);

                }
                break;
            case 3:
                if(check == true){
                    setPathState(-1);
                }
                else if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    Servo_kan(0.2);
                    wrist.setPosition(1);
                    neep.setPosition(0);
                    track();
                    follower.followPath(keep52);
                    setPathState(4);

                }
                break;
            case 4:
                if(check == true){
                    setPathState(-1);
                }
                else if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    Servo_kan(0.2);
                    wrist.setPosition(1);
                    neep.setPosition(0);
                    track();
                    follower.followPath(keep53);
                    setPathState(5);

                }
                break;
            case 5:
                if(check == true){
                    setPathState(-1);
                }
                else if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    Servo_kan(0.2);
                    wrist.setPosition(1);
                    neep.setPosition(0);
                    track();


                    follower.followPath(keep54);
                    setPathState(909);

                }
                break;
            case 909:
                if(check == true){
                    setPathState(-1);
                }
                else if (!follower.isBusy()) {
                    P3.setPosition(0.18);
                    Thread.sleep(1000);
                    setPathState(-1);

                }
                break;
        }
    }

    public void mecpath() {
        switch (mec) {
            case 0:
                upliftset(1450);
                break;
            case 1:
                downlift();
                break;
        }
    }
    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setMec(int pMec) {
        mec = pMec;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {


        // These loop the movements of the robot
        follower.update();

        try {
            autonomousPathUpdate();
            mecpath();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("lift1",L1.getCurrentPosition());
        telemetry.addData("lift2",L2.getCurrentPosition());
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        wrist = hardwareMap.get(Servo.class, "wrist");
        neep = hardwareMap.get(Servo.class, "neep");
        spin = hardwareMap.get(Servo.class, "spin");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        P3 = hardwareMap.get(Servo.class, "P3");
        wrist.setDirection(Servo.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
//        myColorBlobLocatorProcessorBuilder.setTargetColorRange(new ColorRange(ColorSpace.RGB, new Scalar(30, 30, 10), new Scalar(200, 255, 25)));
        myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.YELLOW);

        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(0.5, 0.6, 1, -0.2));

        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);

        myColorBlobLocatorProcessorBuilder.setDrawContours(false);

        myColorBlobLocatorProcessorBuilder.setBlurSize(5);

        myColorBlobLocatorProcessorBuilder.setErodeSize(5);

        myColorBlobLocatorProcessorBuilder.setDilateSize(5);
        myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
        // Build a vision portal to run the Color Locator process.
        myVisionPortalBuilder = new VisionPortal.Builder();
        //  - Add the ColorBlobLocatorProcessor created above.
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        //  - Set the desired video resolution.
        //      Since a high resolution will not improve this process, choose a lower resolution that is
        //      supported by your camera. This will improve overall performance and reduce latency.
        myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
        //  - Choose your video source. This may be for a webcam or for a Phone Camera.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();
        data_right = JavaUtil.createListWith();
        // Speed up telemetry updates, Just use for debugging.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        Servo_kan(0.2);
        wrist.setPosition(1);
        neep.setPosition(0);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setMec(-1);

    }
    public void track() throws InterruptedException {
        check = false;
        time2 = System.currentTimeMillis();
        while ((System.currentTimeMillis() - time2) <= 100) {
            myBlobs = myColorBlobLocatorProcessor.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(200, 7000, myBlobs);
            for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
                check = true;
                myBlob = myBlob_item;
                myBoxFit = myBlob.getBoxFit();
                mySize = myBoxFit.boundingRect();
                {
                    myPoints = new org.opencv.core.Point[4];
                    myBoxFit.points(myPoints);
                }
                if (mySize.width < mySize.height) {
                    spin.setPosition(0);
                } else {
                    spin.setPosition(0.5);
                }
                Thread.sleep(200);
                Servo_kan(0);
                Thread.sleep(500);
                neep.setPosition(1);
                Thread.sleep(400);

            }
        }
    }


    private void Servo_kan(double right) {
        rightservo.setPosition(right);
        leftservo.setPosition(1 - right);
    }



    private void upliftset(int up) {
        wrist.setPosition(0.43);
        spin.setPosition(0);
        if (L2.getCurrentPosition() < up) {
            Servo_kan(0.4);
            L1.setPower(1);
            L2.setPower(1);
        } else {
            Servo_kan(0.51);
            L1.setPower(0.05);
            L2.setPower(0.05);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setMec(-1);
        }
    }
    private void downlift(){
        if (Math.abs(L2.getCurrentPosition()) >= 25) {
            L1.setPower(-1);
            L2.setPower(-1);
        }
        else {
            L1.setPower(0);
            L2.setPower(0);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setMec(-1);
        }

    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        ;
    }
}
