package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

@Autonomous(name = "AutoSpecimen_RED")
public class SPECIMEN_AUTONOMOUS_RED extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int mec;
    private Servo rightservo;
    private Servo leftservo;
    private Servo neep;
    private Servo spin;
    private Servo wrist;
    private DcMotor L2 ;
    private DcMotor L1 ;
    private Servo OPEN;


    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
//    public ClawSubsystem claw;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose hang_Sp1 = new Pose(26.5, 0, Math.toRadians(0));
    private final Pose Slide_Sp1 = new Pose(24, -25, Math.toRadians(0));
    private final Pose pl_Sp1_FW = new Pose(50, -32, Math.toRadians(0));
    private final Pose pl_Sp1_Slide = new Pose(50, -39, Math.toRadians(0));
    private final Pose pl_Sp1_Back = new Pose(16, -39, Math.toRadians(0));
    private final Pose pl_Sp2_FW = new Pose(50, -44, Math.toRadians(0));
    private final Pose pl_Sp2_Slide = new Pose(50, -48, Math.toRadians(0));
    private final Pose pl_Sp2_Back = new Pose(16, -48, Math.toRadians(0));
    private final Pose pl_Sp3_FW = new Pose(50, -54, Math.toRadians(0));
    private final Pose pl_Sp3_Slide = new Pose(50, -57, Math.toRadians(0));
    private final Pose human_Sp2 = new Pose(12, -57, Math.toRadians(0));
    private final Pose hang_Sp2 = new Pose(25.5, 2, Math.toRadians(0));
    private final Pose keep_Sp3 = new Pose(10.8, -30, Math.toRadians(0));
    private final Pose hang_Sp3 = new Pose(25.5, 1, Math.toRadians(0));
    private final Pose hang_Sp4 = new Pose(25.5, -1, Math.toRadians(0));
    private final Pose hang_Sp5 = new Pose(25.5, -2.5, Math.toRadians(0));
    private final Pose hang_Sp_back = new Pose(14, -2, Math.toRadians(0));
    private final Pose End = new Pose(15, 30, Math.toRadians(-80));



    private Path hang_preload, park;
    private PathChain Point1_hang, Point2_Sp2FW, Point3_Sp2Sl, Point4_Sp2Back,  Point5_Sp3FW, Point6_Sp3Sl,Point7_Sp3Ba,Point8_Sp4FW,Point9_Sp4Sl,Point10_Sp4keep,hangSp2,keepSp3,hangSp3,hangSp4,hangSp5,keepSp1,keepSp2,Finish,hangSp5back,keep_sample;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        hang_preload = new Path(new BezierLine(new Point(startPose), new Point(hang_Sp1)));
        hang_preload.setLinearHeadingInterpolation(startPose.getHeading(), hang_Sp1.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Point1_hang = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp1), new Point(Slide_Sp1)))
                .setLinearHeadingInterpolation(hang_Sp1.getHeading(), Slide_Sp1.getHeading())
                .build();

        Point2_Sp2FW = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Slide_Sp1), new Point(pl_Sp1_FW)))
                .setLinearHeadingInterpolation(Slide_Sp1.getHeading(), pl_Sp1_FW.getHeading())
                .build();

        Point3_Sp2Sl = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp1_FW), new Point(pl_Sp1_Slide)))
                .setLinearHeadingInterpolation(pl_Sp1_FW.getHeading(), pl_Sp1_Slide.getHeading())
                .build();

        Point4_Sp2Back = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp1_Slide), new Point(pl_Sp1_Back)))
                .setLinearHeadingInterpolation(pl_Sp1_Slide.getHeading(), pl_Sp1_Back.getHeading())
                .build();

        Point5_Sp3FW = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp1_Back), new Point(pl_Sp2_FW)))
                .setLinearHeadingInterpolation(pl_Sp1_Back.getHeading(), pl_Sp2_FW.getHeading())
                .build();

        Point6_Sp3Sl = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp2_FW), new Point(pl_Sp2_Slide)))
                .setLinearHeadingInterpolation(pl_Sp2_FW.getHeading(), pl_Sp2_Slide.getHeading())
                .build();

        Point7_Sp3Ba = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp2_Slide), new Point(pl_Sp2_Back)))
                .setLinearHeadingInterpolation(pl_Sp2_Slide.getHeading(), pl_Sp2_Back.getHeading())
                .build();

        Point8_Sp4FW = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp2_Back), new Point(pl_Sp3_FW)))
                .setLinearHeadingInterpolation(pl_Sp2_Back.getHeading(), pl_Sp3_FW.getHeading())
                .build();

        Point9_Sp4Sl = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp3_FW), new Point(pl_Sp3_Slide)))
                .setLinearHeadingInterpolation(pl_Sp3_FW.getHeading(), pl_Sp3_Slide.getHeading())
                .build();

        Point10_Sp4keep = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp3_Slide), new Point(human_Sp2)))
                .setLinearHeadingInterpolation(pl_Sp3_Slide.getHeading(), human_Sp2.getHeading())
                .build();

        hangSp2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(human_Sp2),new Point(hang_Sp_back) ,new Point(hang_Sp2)))
                .setLinearHeadingInterpolation(human_Sp2.getHeading(), hang_Sp2.getHeading())
                .build();

        keepSp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp2), new Point(keep_Sp3)))
                .setLinearHeadingInterpolation(hang_Sp2.getHeading(), keep_Sp3.getHeading())
                .build();

        hangSp3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_Sp3),new Point(hang_Sp_back), new Point(hang_Sp3)))
                .setLinearHeadingInterpolation(keep_Sp3.getHeading(), hang_Sp3.getHeading())
                .build();

        keepSp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp3), new Point(keep_Sp3)))
                .setLinearHeadingInterpolation(hang_Sp3.getHeading(), keep_Sp3.getHeading())
                .build();

        hangSp4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_Sp3),new Point(hang_Sp_back), new Point(hang_Sp4)))
                .setLinearHeadingInterpolation(keep_Sp3.getHeading(), hang_Sp4.getHeading())
                .build();

        keepSp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp4), new Point(keep_Sp3)))
                .setLinearHeadingInterpolation(hang_Sp4.getHeading(), keep_Sp3.getHeading())
                .build();

        hangSp5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_Sp3),new Point(hang_Sp_back), new Point(hang_Sp5)))
                .setLinearHeadingInterpolation(keep_Sp3.getHeading(), hang_Sp5.getHeading())
                .build();

        keep_sample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hang_Sp5),new Point(keep_Sp3)))
                .setLinearHeadingInterpolation(hang_Sp5.getHeading(), keep_Sp3.getHeading())
                .build();

        Finish = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_Sp3),new Point(End)))
                .setLinearHeadingInterpolation(keep_Sp3.getHeading(), End.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:

                setMec(0);
                follower.setMaxPower(0.7);
                follower.followPath(hang_preload , true);
                neep.setPosition(1);
                spin.setPosition(0);
                setPathState(101);



                break;
            case 101:
                if(follower.getPose().getX() > (hang_Sp1.getX() - 1) && follower.getPose().getY() > (hang_Sp1.getY() - 1)) {
                    wrist.setPosition(1);
                    neep.setPosition(0);
                    Thread.sleep(250);
                    setPathState(1);
                }
                break;
            case 1:

                if(follower.getPose().getX() > (hang_Sp1.getX() - 1) && follower.getPose().getY() > (hang_Sp1.getY() - 1)) {
                    follower.setMaxPower(0.9);
                    Servo_kan(1);
                    Thread.sleep(250);
                    setMec(2);
                    follower.followPath(Point1_hang,true);
                    setPathState(2);

                }
                break;
            case 2:

                follower.setMaxPower(1);
                if(follower.getPose().getX() < (Slide_Sp1.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(Slide_Sp1.getY()) - 1) {
                    follower.followPath(Point2_Sp2FW,true);
                    setPathState(3);

                }
                break;
            case 3:

                if(follower.getPose().getX() > (pl_Sp1_FW.getX() - 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp1_FW.getY()) -1) {
                    follower.setMaxPower(1);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Point3_Sp2Sl,true);

                    setPathState(4);

                }
                break;
            case 4:
                if (follower.getPose().getX() > (pl_Sp1_Slide.getX() - 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp1_Slide.getY()) - 1)  {
                    follower.setMaxPower(1);
                    follower.followPath(Point4_Sp2Back,true);

                    setPathState(5);

                }
                break;
            case 5:


                if (follower.getPose().getX() < (pl_Sp1_Back.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp1_Back.getY())-1) {
                    follower.setMaxPower(1);
                    follower.followPath(Point5_Sp3FW,true);

                    setPathState(6);

                }
                break;
            case 6:

                if(follower.getPose().getX() > (pl_Sp2_FW.getX() - 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp2_FW.getY()) - 1) {
                    follower.setMaxPower(1);
                    follower.followPath(Point6_Sp3Sl,true);

                    setPathState(7);

                }
                break;
            case 7:
                if(follower.getPose().getX() > (pl_Sp2_Slide.getX() - 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp2_Slide.getY()) - 1) {
                    follower.setMaxPower(1);
                    follower.followPath(Point7_Sp3Ba,true);

                    setPathState(8);

                }
                break;
            case 8:


                if(follower.getPose().getX() < (pl_Sp2_Back.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp2_Back.getY()) - 1 ) {
                    follower.setMaxPower(1);
                    spin.setPosition(0);
                    wrist.setPosition(0.28);
                    follower.followPath(Point8_Sp4FW,true);

                    setPathState(9);

                }
                break;
            case 9:
                if(follower.getPose().getX() > (pl_Sp3_FW.getX() - 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp3_FW.getY()) - 1) {
                    follower.setMaxPower(1);
                    follower.followPath(Point9_Sp4Sl,true);


                    setPathState(10);

                }
                break;
            case 10:
                if(follower.getPose().getX() < (pl_Sp3_Slide.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(pl_Sp3_Slide.getY()) - 1) {
                    follower.setMaxPower(0.65);
                    spin.setPosition(0);
                    follower.followPath(Point10_Sp4keep,true);

                    setPathState(102);

                }
                break;

            case 102:
                if(!follower.isBusy()) {
                    neep.setPosition(1);
                    Thread.sleep(250);
                    setMec(0);
                    neep.setPosition(1);
                    spin.setPosition(0);
                    setPathState(11);
                }
                break;

            case 11:
                if (follower.getPose().getX() < (human_Sp2.getX() + 0.1) && Math.abs(follower.getPose().getY()) > Math.abs(human_Sp2.getY()) - 0.1) {
                    follower.setMaxPower(1);
                    follower.followPath(hangSp2,true);
                    setPathState(103);
                }
                break;
            case 103:
                if(!follower.isBusy()) {
                    wrist.setPosition(1);
                    spin.setPosition(0);
                    neep.setPosition(0);
                    Thread.sleep(100);
                    setPathState(12);
                }
                break;
            case 12:
                if(follower.getPose().getX() > (hang_Sp2.getX() - 1) && Math.abs(follower.getPose().getY()) < Math.abs(hang_Sp2.getY()) + 1) {
                    follower.setMaxPower(1);
                    follower.followPath(keepSp3,true);
                    Servo_kan(1);
                    Thread.sleep(150);
                    setMec(2);
                    wrist.setPosition(0.28);
                    setPathState(104);
                }
                break;
            case 104:
                if(!follower.isBusy()) {
                    neep.setPosition(1);
                    Thread.sleep(250);
                    setMec(0);
                    neep.setPosition(1);
                    spin.setPosition(0);
                    setPathState(13);
                }
                break;
            case 13:
                if((follower.getPose().getX() < (keep_Sp3.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(keep_Sp3.getY()) - 1)) {
                    follower.setMaxPower(1);
                    follower.followPath(hangSp3,true);
                    setPathState(105);
                }
                break;
            case 105:
                if(!follower.isBusy())  {
                    wrist.setPosition(1);
                    spin.setPosition(0);
                    neep.setPosition(0);
                    Thread.sleep(200);
                    Servo_kan(1);
                    Thread.sleep(150);
                    setMec(2);
                    setPathState(14);
                }
                break;
            case 14:
                if(follower.getPose().getX() > (hang_Sp3.getX() - 1) && Math.abs(follower.getPose().getY()) < Math.abs(hang_Sp3.getY()) + 1)  {
                    follower.setMaxPower(1);
                    wrist.setPosition(0.28);
                    Servo_kan(1);
                    Thread.sleep(200);
                    spin.setPosition(0);
                    follower.followPath(keepSp1,true);
                    setPathState(106);
                }
                break;

            case 106:
                if(!follower.isBusy()){
                    neep.setPosition(1);
                    Thread.sleep(250);
                    setMec(0);
                    neep.setPosition(1);
                    spin.setPosition(0);
                    setPathState(15);
                }
                break;
            case 15:
                if((follower.getPose().getX() < (keep_Sp3.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(keep_Sp3.getY()) - 1)) {
                    follower.setMaxPower(1);
                    follower.followPath(hangSp4,true);
                    setPathState(108);
                }
                break;
            case 108:
                if(!follower.isBusy())   {
                    wrist.setPosition(1);
                    spin.setPosition(0);
                    neep.setPosition(0);
                    Thread.sleep(200);
                    Servo_kan(1);
                    Thread.sleep(150);
                    setMec(2);
                    setPathState(16);
                }
                break;

            case 16:
                if(follower.getPose().getX() > (hang_Sp4.getX() - 1) && Math.abs(follower.getPose().getY()) < Math.abs(hang_Sp4.getY()) + 1)  {
                    follower.setMaxPower(1);
                    wrist.setPosition(0.28);
                    Servo_kan(1);
                    Thread.sleep(200);
                    spin.setPosition(0);
                    follower.followPath(keepSp2,true);
                    setPathState(109);
                }
                break;

            case 109:
                if(!follower.isBusy()) {
                    neep.setPosition(1);
                    Thread.sleep(250);
                    setMec(0);
                    neep.setPosition(1);
                    spin.setPosition(0);
                    setPathState(17);
                }
                break;
            case 17:
                if((follower.getPose().getX() < (keep_Sp3.getX() + 1) && Math.abs(follower.getPose().getY()) > Math.abs(keep_Sp3.getY()) - 1)) {
                    follower.setMaxPower(1);
                    follower.followPath(hangSp5,true);
                    setPathState(110);
                }
                break;
            case 110:
                if(!follower.isBusy()) {
                    wrist.setPosition(1);
                    spin.setPosition(0);
                    neep.setPosition(0);
                    Thread.sleep(200);
                    Servo_kan(1);
                    Thread.sleep(150);
                    setMec(2);
                    setPathState(18);
                }
                break;
            case 18:
                if(follower.getPose().getX() > (hang_Sp5.getX() - 1) && Math.abs(follower.getPose().getY()) < Math.abs(hang_Sp5.getY()) + 1)  {
                    wrist.setPosition(1);
                    Servo_kan(1);
                    Thread.sleep(200);
                    spin.setPosition(0);
                    follower.followPath(keep_sample,true);
                    setPathState(111);
                }
                break;

        }
    }
    public void mecpath() throws InterruptedException{
        switch (mec) {
            case 0:
                uplifthang(630);
                break;
            case 1:
                uplifthang(350);
                break;
            case 2:
                downlift();
                break;
            case 3:
                upliftset(1450);
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
        telemetry.update();
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
        OPEN = hardwareMap.get(Servo.class, "OPEN");
        wrist.setDirection(Servo.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OPEN.setPosition(0);
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
    }

    private void Servo_kan(double right) {
        rightservo.setPosition(right);
        leftservo.setPosition(1 - right);
    }


    private void uplifthang(int up) {

        if (L2.getCurrentPosition() < up) {
            L1.setPower(1);
            L2.setPower(1);
            Servo_kan(0.28);
            spin.setPosition(0);
            neep.setPosition(1);
            wrist.setPosition(1);

        } else {

            L1.setPower(0.08);
            L2.setPower(0.08);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setMec(-1);

        }




    }
    private void upliftset(int up) {
        wrist.setPosition(0.36);
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
        if (Math.abs(L2.getCurrentPosition()) >= 15) {
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
    /**

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
