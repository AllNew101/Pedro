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


@Autonomous(name = "S5")
public class S5 extends OpMode {

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
    private boolean debug = false;
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    ColorBlobLocatorProcessor.Blob myBlob;
    RotatedRect myBoxFit;
    org.opencv.core.Rect mySize;
    boolean vertical;
    org.opencv.core.Point[] myPoints;
    double angle = 0;
    double count = 0;
    double area = 6000;
    double Perfect_X;
    double Perfect_Y;
    List data_right;
    List data_middle;

    double middle_y = 0;
    double middle_x = 0;
    double right_x = 0;
    double right_y = 0;


    private Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose keep_S5 = new Pose(0,0,Math.toRadians(0));





    private Path Point1_set;
    private PathChain keepS2, keepS3, Point2_set,  Point3_set, keepS4,Point4_set,finale,Point9_Sp4Sl,Point10_Sp4keep,hangSp2,keepSp3,hangSp3,hangSp4,hangSp5,keepSp1,keepSp2,Finish,keep5;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        keep5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(keep_S5)))
                .setLinearHeadingInterpolation(startPose.getHeading(), keep_S5.getHeading())
                .build();



    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
            case 80:
                if (!follower.isBusy()) {
                    startPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0));
                    follower.setMaxPower(1);

                    Servo_kan(0.2);
                    wrist.setPosition(1);
                    neep.setPosition(0.2);
                    track();

                    keep5 = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(startPose), new Point(keep_S5)))
                            .setLinearHeadingInterpolation(startPose.getHeading(), keep_S5.getHeading())
                            .build();


                }
                break;
            case 801:
                if (!follower.isBusy()) {

                    if (check == true){
                        setPathState(802);
                        follower.followPath(keep5);}
                    else if (count == 0){
                        setPathState(80);
                        follower.followPath(keep5);
                        count++;
                    }
                    else if (count == 1) {
                        area = 10000;
                        setPathState(80);
                        count++;
                    }
                    else if (count == 1) {
                        area = 40000;
                        setPathState(80);
                        count++;
                    }

                    else {
                        setPathState(803);
                    }
                }
                break;
            case 802:
                if (!follower.isBusy()) {
                    Servo_kan(0.08);
                    Thread.sleep(500);
                    Servo_kan(0.05);
                    Thread.sleep(400);
                    neep.setPosition(1);
                    Thread.sleep(400);
                    setPathState(-1);
                }
                break;
            case 803:
                P3.setPosition(0.16);
                Thread.sleep(1000);
                setPathState(-1);
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

        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1));

        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        myColorBlobLocatorProcessorBuilder.setDrawContours(false);
        myColorBlobLocatorProcessorBuilder.setBlurSize(5);
        myColorBlobLocatorProcessorBuilder.setErodeSize(5);
        myColorBlobLocatorProcessorBuilder.setDilateSize(5);
        myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
        // Build a vision portal to run the Color Locator process.
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();
        data_right = JavaUtil.createListWith();
        data_middle = JavaUtil.createListWith();
        Perfect_Y = 137.0;
        Perfect_X = 280.0;
        // Speed up telemetry updates, Just use for debugging.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        Servo_kan(0.2);
        wrist.setPosition(1);
        neep.setPosition(0.2);
        spin.setPosition(0);
    }

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
        data_right.removeAll(data_right);
        data_middle.removeAll(data_middle);

        time2 = System.currentTimeMillis();
        while ((System.currentTimeMillis() - time2) <= 300) {
            myBlobs = myColorBlobLocatorProcessor.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(2200, 10000, myBlobs);
            for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
                myBlob = myBlob_item;
                myBoxFit = myBlob.getBoxFit();
                mySize = myBoxFit.boundingRect();
                {
                    myPoints = new org.opencv.core.Point[4];
                    myBoxFit.points(myPoints);
                }
                if (mySize.width / mySize.height > 1) {
                    vertical = false;
                } else if (mySize.width / mySize.height < 1){
                    vertical = true;
                }

                if (myBoxFit.center.x >= 240 && myBoxFit.center.y >= 90 && myBlob.getContourArea() <= area) {
                    data_right.add(JavaUtil.createListWith(Math.round(Math.sqrt(Math.pow(Perfect_X - myBoxFit.center.x, 2) + Math.pow(Perfect_Y - myBoxFit.center.y, 2))), vertical, myBoxFit.center.x, myBoxFit.center.y));
                }else if (myBoxFit.center.x >= 240 && myBoxFit.center.y >= 90 && myBlob.getContourArea() > area){
                    Servo_kan(0);
                    neep.setPosition(1);
                    Thread.sleep(400);
                    neep.setPosition(0);
                    Thread.sleep(400);
                    Servo_kan(0.2);
                    Thread.sleep(400);
                    }
                 else if (myBoxFit.center.x >= 80 && myBoxFit.center.y >= 90 && myBlob.getContourArea() <= area) {
                    data_middle.add(JavaUtil.createListWith(Math.round(Math.sqrt(Math.pow(Perfect_X - myBoxFit.center.x, 2) + Math.pow(Perfect_Y - myBoxFit.center.y, 2))), vertical, myBoxFit.center.x, myBoxFit.center.y));
                }
            }
            if (JavaUtil.listLength(data_right) + JavaUtil.listLength(data_middle) >= 1) {
                data_right = JavaUtil.sort(data_right, JavaUtil.SortType.NUMERIC, JavaUtil.SortDirection.ASCENDING);
                data_middle = JavaUtil.sort(data_middle, JavaUtil.SortType.NUMERIC, JavaUtil.SortDirection.ASCENDING);
                break;
            }
        }
        telemetry.addData("Array", data_right + " ||| " + data_middle);


        if (JavaUtil.listLength(data_right) != 0) {
            if (((Boolean) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 1, false)).booleanValue()) {
                spin.setPosition(0);
                right_x = 0.5;
                right_y = 0.7;
            } else {
                spin.setPosition(0.5);
                right_x = 0.5;
                right_y = 0.7;
            }
            Thread.sleep(200);

            if ((Long)JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 0, false) <= 20) {
                check = true;
                setPathState(102);
            } else {
                check = true;
                setPathState(101);
                adjust_y = ((Perfect_Y - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue()) * 0.1167 * right_y) * 0.5;
                adjust_x = ((Perfect_X - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue()) * 0.046875 * right_x) * 0.71;
                telemetry.addData("x_right", adjust_x);
                telemetry.addData("y_right", adjust_y);
                telemetry.update();

                keep_S5 = new Pose(follower.getPose().getX() + adjust_y ,follower.getPose().getY() + adjust_x ,Math.toRadians(0));

                if (debug == true) {Thread.sleep(1000);}
            }

        } else if (JavaUtil.listLength(data_middle) != 0){
            check = true;
            setPathState(101);
            if (((Boolean) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_middle, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 1, false)).booleanValue()) {
                spin.setPosition(0);
                middle_x = 0.5;
                middle_y = 0.45;
            } else {
                spin.setPosition(0.5);
                middle_x = 0.45;
                middle_y = 0.5;
            }
            Thread.sleep(200);
            adjust_y = ((Perfect_Y - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_middle, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue()) * 0.1167 * middle_y) * 0.5;
            adjust_x = ((Perfect_X - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_middle, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue()) * 0.046875 * middle_x) * 0.71;
            telemetry.addData("x_middle", adjust_x);
            telemetry.addData("y_middle", adjust_y);
            telemetry.update();

            keep_S5 = new Pose(follower.getPose().getX() + adjust_y ,follower.getPose().getY() + adjust_x ,Math.toRadians(0));

            if (debug == true) {Thread.sleep(1000);}
        }
        else {

            keep_S5 = new Pose(follower.getPose().getX() ,follower.getPose().getY() + 3 ,Math.toRadians(0));
            setPathState(101);
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
