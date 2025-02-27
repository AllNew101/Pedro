package org.firstinspires.ftc.teamcode.opmode.auto;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "test01", group = "Examples")
public class test01 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int mec;
    private Servo RServo;
    private Servo LServo;
    private Servo Kap_Hand;
    private Servo EServo;
    private DcMotor L2 ;
    private DcMotor L1 ;



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


    private final Pose hang_Sp1 = new Pose(19, 0, Math.toRadians(0));


    private final Pose Slide = new Pose(14, -18, Math.toRadians(0));


    private final Pose pl_Sp1 = new Pose(33, -21, Math.toRadians(0));
    private final Pose pl_Sp1_Slide = new Pose(33, -25, Math.toRadians(0));
    private final Pose pl_Sp1_Back = new Pose(10, -25, Math.toRadians(0));
    private final Pose pl_Sp2_FW = new Pose(33, -28, Math.toRadians(0));
    private final Pose pl_Sp2_Slide = new Pose(33, -32, Math.toRadians(0));
    private final Pose pl_Sp2_Back = new Pose(10, -32, Math.toRadians(0));
    private final Pose pl_Sp3_FW = new Pose(33, -36, Math.toRadians(0));
    private final Pose pl_Sp3_Slide = new Pose(33, -36, Math.toRadians(0));
    private final Pose pl_Sp3_Back = new Pose(10, -36, Math.toRadians(0));
    private final Pose human = new Pose(3, -36, Math.toRadians(0));
    private final Pose hang_Sp2 = new Pose(18, -1, Math.toRadians(0));
    private final Pose hang_Sp3 = new Pose(18, -0.8, Math.toRadians(0));
    private final Pose hang_Sp4 = new Pose(18, -0.6, Math.toRadians(0));
    private final Pose keep_Sp = new Pose(4.5, -15, Math.toRadians(0));


    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, grabfood,  grabPickup5, grabPickup6,grabPickup7,grabPickup8,grabPickup9,grabPickup10,hangSp2,keepSp,hangSp3,hangSp4,keepSp1;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(hang_Sp1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), hang_Sp1.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp1), new Point(Slide)))
                .setLinearHeadingInterpolation(hang_Sp1.getHeading(), Slide.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Slide), new Point(pl_Sp1)))
                .setLinearHeadingInterpolation(Slide.getHeading(), pl_Sp1.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp1), new Point(pl_Sp1_Slide)))
                .setLinearHeadingInterpolation(pl_Sp1.getHeading(), pl_Sp1_Slide.getHeading())
                .build();
        grabfood = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp1_Slide), new Point(pl_Sp1_Back)))
                .setLinearHeadingInterpolation(pl_Sp1_Slide.getHeading(), pl_Sp1_Back.getHeading())
                .build();
        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp1_Back), new Point(pl_Sp2_FW)))
                .setLinearHeadingInterpolation(pl_Sp1_Back.getHeading(), pl_Sp2_FW.getHeading())
                .build();

        grabPickup6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp2_FW), new Point(pl_Sp2_Slide)))
                .setLinearHeadingInterpolation(pl_Sp2_FW.getHeading(), pl_Sp2_Slide.getHeading())
                .build();
        grabPickup7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp2_Slide), new Point(pl_Sp2_Back)))
                .setLinearHeadingInterpolation(pl_Sp2_Slide.getHeading(), pl_Sp2_Back.getHeading())
                .build();
        grabPickup8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp2_Back), new Point(pl_Sp3_FW)))
                .setLinearHeadingInterpolation(pl_Sp2_Back.getHeading(), pl_Sp3_FW.getHeading())
                .build();
        grabPickup9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp3_FW), new Point(pl_Sp3_Slide)))
                .setLinearHeadingInterpolation(pl_Sp3_FW.getHeading(), pl_Sp3_Slide.getHeading())
                .build();
        grabPickup10 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pl_Sp3_Slide), new Point(human)))
                .setLinearHeadingInterpolation(pl_Sp3_Slide.getHeading(), human.getHeading())
                .build();

        hangSp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(human), new Point(hang_Sp2)))
                .setLinearHeadingInterpolation(human.getHeading(), hang_Sp2.getHeading())
                .build();

        keepSp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp2), new Point(keep_Sp)))
                .setLinearHeadingInterpolation(hang_Sp2.getHeading(), keep_Sp.getHeading())
                .build();

        hangSp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(keep_Sp), new Point(hang_Sp3)))
                .setLinearHeadingInterpolation(keep_Sp.getHeading(), hang_Sp3.getHeading())
                .build();
        keepSp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hang_Sp3), new Point(keep_Sp)))
                .setLinearHeadingInterpolation(hang_Sp3.getHeading(), keep_Sp.getHeading())
                .build();
        hangSp4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(keep_Sp), new Point(hang_Sp4)))
                .setLinearHeadingInterpolation(keep_Sp.getHeading(), hang_Sp4.getHeading())
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.6);
                follower.followPath(scorePreload);
                EServo.setPosition(0);
                uplifthang(400);
                setPathState(101);


                break;
            case 101:
                if(follower.getPose().getX() > (hang_Sp1.getX() - 1) && follower.getPose().getY() > (hang_Sp1.getY() - 1)) {
                    follower.followPath(scorePreload);

                    EServo.setPosition(1);
                    Servo_kan(0.58);
                    Thread.sleep(700);
                    setPathState(1);
                }
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (hang_Sp1.getX() - 1) && follower.getPose().getY() > (hang_Sp1.getY() - 1)) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);

                }
                break;
            case 2:
                follower.setMaxPower(1);
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (Slide.getX() - 1) && follower.getPose().getY() > (Slide.getY() - 1)) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);

                    setPathState(3);

                }
                break;
            case 3:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (pl_Sp1.getX() - 1) && follower.getPose().getY() > (pl_Sp1.getY() - 1)) {
                    follower.setMaxPower(0.8);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);

                    setPathState(4);

                }
                break;
            case 4:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabfood,true);

                    setPathState(5);

                }
                break;
            case 5:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup5,true);
                    L1.setPower(-0.4);
                    L2.setPower(-0.4);
                    Servo_kan(0.6);
                    Kap_Hand.setPosition(0.24);

                    setPathState(6);

                }
                break;
            case 6:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (pl_Sp2_FW.getX() - 1) && follower.getPose().getY() > (pl_Sp2_FW.getY() - 1)) {
                    follower.setMaxPower(0.9);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup6,false);

                    setPathState(7);

                }
                break;
            case 7:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (pl_Sp2_Slide.getX() - 1) && follower.getPose().getY() > (pl_Sp2_Slide.getY() - 1)) {
                    follower.setMaxPower(0.95);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup7,true);

                    setPathState(8);

                }
                break;
            case 8:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup8,true);

                    setPathState(10);

                }
                break;
            case 9:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup9,true);


                    setPathState(10);

                }
                break;
            case 10:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup10,true);

                    setPathState(102);

                }
                break;

            case 102:
                if(!follower.isBusy()) {
                    EServo.setPosition(0);
                    Thread.sleep(400);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(hangSp2,false);

                    uplifthang(350);
                    setPathState(103);
                }
                break;
            case 103:
                if(!follower.isBusy()) {
                    L1.setPower(0.9);
                    L2.setPower(0.9);
                    Thread.sleep(250);
                    L1.setPower(0);
                    L2.setPower(0);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    EServo.setPosition(1);
                    Servo_kan(0.6);
                    Kap_Hand.setPosition(0.19);
                    L1.setPower(-0.45);
                    L2.setPower(-0.45);
                    Thread.sleep(100);
                    follower.followPath(keepSp,true);
                    setPathState(104);
                }
                break;
            case 104:
                if(!follower.isBusy()) {
                    EServo.setPosition(0);
                    Thread.sleep(400);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(hangSp3,false);
                    L1.setPower(0);
                    L2.setPower(0);
                    uplifthang(350);
                    setPathState(105);
                }
                break;
            case 105:
                if(!follower.isBusy()) {
                    L1.setPower(0.9);
                    L2.setPower(0.9);
                    Thread.sleep(250);
                    L1.setPower(0);
                    L2.setPower(0);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    EServo.setPosition(1);
                    Servo_kan(0.6);
                    Kap_Hand.setPosition(0.19);
                    L1.setPower(-0.45);
                    L2.setPower(-0.45);
                    Thread.sleep(100);
                    follower.followPath(keepSp1,true);
                    setPathState(106);
                }
                break;

            case 106:
                if(!follower.isBusy()) {
                    EServo.setPosition(0);
                    Thread.sleep(400);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(hangSp4,false);
                    L1.setPower(0);
                    L2.setPower(0);
                    uplifthang(350);
                    setPathState(108);
                }
                break;
            case 108:
                if(!follower.isBusy()) {
                    L1.setPower(0.9);
                    L2.setPower(0.9);
                    Thread.sleep(250);
                    L1.setPower(0);
                    L2.setPower(0);
                    setPathState(16);
                }
                break;
        }
    }
    public void mecpath() throws InterruptedException{
        switch (mec) {
            case 0:
                uplifthang(400);
            case 1:
                uplifthang(350);
            case 2:
                L1.setPower(-0.45);
                L2.setPower(-0.45);
                Thread.sleep(450);
                L1.setPower(0);
                L2.setPower(0);

        }}
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
        RServo = hardwareMap.get(Servo.class, "RServo");
        LServo = hardwareMap.get(Servo.class, "LServo");
        Kap_Hand = hardwareMap.get(Servo.class, "Kap_Hand");
        EServo = hardwareMap.get(Servo.class, "EServo");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        Kap_Hand.setDirection(Servo.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setDirection(DcMotor.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        claw = new ClawSubsystem(hardwareMap);
//
//        // Set the claw to positions for init
//        claw.closeClaw();
//        claw.startClaw();
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
        RServo.setPosition(1 - right);
        LServo.setPosition(right);
    }

    private void uplifthang(int up) {

            if ((L1.getCurrentPosition() + L2.getCurrentPosition()) / 2 <= up) {
                L1.setPower(0.9);
                L2.setPower(0.9);


            } else if ((L1.getCurrentPosition() + L2.getCurrentPosition()) / 2 > up) {
                Servo_kan(0.3);
                Kap_Hand.setPosition(1);
                L1.setPower(0);
                L2.setPower(0);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }



    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
