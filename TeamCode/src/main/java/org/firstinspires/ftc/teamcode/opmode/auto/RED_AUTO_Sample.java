package org.firstinspires.ftc.teamcode.opmode.auto;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "AutoSamples_RED")
public class RED_AUTO_Sample extends OpMode {

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
    private Servo P3;
    private DcMotor L2 ;
    private DcMotor L1 ;



    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose set_Sample = new Pose(-18, 4, Math.toRadians(45));
    private final Pose keep_S2 = new Pose(-11, 11.8, Math.toRadians(90));
    private final Pose set_Sample2 = new Pose(-18, 8, Math.toRadians(23));
    private final Pose keep_S3 = new Pose(-18.3, 11.5, Math.toRadians(90));
    private final Pose set_Sample3 = new Pose(-20, 6.5, Math.toRadians(28));
    private final Pose keep_S4 = new Pose(-18.8, 12.8, Math.toRadians(114));
    private final Pose set_Sample4 = new Pose(-21.5, 10, Math.toRadians(55));
    private final Pose b_final = new Pose(-21.5, 55, Math.toRadians(55));
    private final Pose final0 = new Pose(12.28,55.72,Math.toRadians(0));





    private Path Point1_set;
    private PathChain keepS2, keepS3, Point2_set,  Point3_set, keepS4,Point4_set,finale,Point9_Sp4Sl,Point10_Sp4keep,hangSp2,keepSp3,hangSp3,hangSp4,hangSp5,keepSp1,keepSp2,Finish;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        Point1_set = new Path(new BezierLine(new Point(startPose), new Point(set_Sample)));
        Point1_set.setLinearHeadingInterpolation(startPose.getHeading(), set_Sample.getHeading());

        keepS2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(set_Sample), new Point(keep_S2)))
                .setLinearHeadingInterpolation(set_Sample.getHeading(), keep_S2.getHeading())
                .build();
        Point2_set = follower.pathBuilder()
                .addPath(new BezierLine(new Point(keep_S2), new Point(set_Sample2)))
                .setLinearHeadingInterpolation(keep_S2.getHeading(), set_Sample2.getHeading())
                .build();
        keepS3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(set_Sample2), new Point(keep_S3)))
                .setLinearHeadingInterpolation(set_Sample2.getHeading(), keep_S3.getHeading())
                .build();
        Point3_set = follower.pathBuilder()
                .addPath(new BezierLine(new Point(keep_S3), new Point(set_Sample3)))
                .setLinearHeadingInterpolation(keep_S3.getHeading(), set_Sample3.getHeading())
                .build();
        keepS4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(set_Sample3), new Point(keep_S4)))
                .setLinearHeadingInterpolation(set_Sample3.getHeading(), keep_S4.getHeading())
                .build();
        Point4_set = follower.pathBuilder()
                .addPath(new BezierLine(new Point(keep_S4), new Point(set_Sample4)))
                .setLinearHeadingInterpolation(keep_S4.getHeading(), set_Sample4.getHeading())
                .build();
        finale = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(set_Sample4),new Point(b_final), new Point(final0)))
                .setLinearHeadingInterpolation(set_Sample4.getHeading(), final0.getHeading())
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                neep.setPosition(1);
                setMec(0);
                follower.followPath(Point1_set);
                setPathState(101);

                break;
            case 101:
                if (!follower.isBusy()) {
                    neep.setPosition(0);
                    Thread.sleep(300);
                    setPathState(1011);
                }
                break;
            case 1011:
                if (!follower.isBusy()) {
                    Servo_kan(0.08);
                    wrist.setPosition(1);
                    setPathState(1);
                }
                break;
            case 1:
                if (Math.abs(follower.getPose().getX()) > Math.abs((set_Sample.getX())) - 1 && follower.getPose().getY() > (set_Sample.getY()) - 1) {
                    follower.setMaxPower(0.8);
                    setMec(1);
                    follower.followPath(keepS2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Servo_kan(0);
                    wrist.setPosition(1);
                    Thread.sleep(250);
                    neep.setPosition(1);
                    Thread.sleep(200);
                    setMec(0);
                    follower.followPath(Point2_set);
                    setPathState(102);
                }
                break;
            case 102:
                if (!follower.isBusy()) {
                    neep.setPosition(0);
                    Thread.sleep(350);
                    setPathState(1021);
                }
                break;
            case 1021:
                if (!follower.isBusy()) {
                    Servo_kan(0.08);
                    wrist.setPosition(1);
                    setPathState(3);
                }
                break;
            case 3:
                if (Math.abs(follower.getPose().getX()) > Math.abs((set_Sample2.getX())) - 1 && Math.abs(follower.getPose().getY()) < Math.abs(set_Sample2.getY()) + 1) {
                    follower.setMaxPower(0.8);
                    setMec(1);
                    follower.followPath(keepS3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Servo_kan(0);
                    wrist.setPosition(1);
                    Thread.sleep(250);
                    neep.setPosition(1);
                    Thread.sleep(200);
                    setMec(0);
                    follower.followPath(Point3_set);
                    setPathState(103);
                }
                break;
            case 103:
                if (!follower.isBusy()) {
                    neep.setPosition(0);
                    Thread.sleep(550);
                    setPathState(1031);
                }
                break;
            case 1031:
                if (!follower.isBusy()) {
                    Servo_kan(0.08);
                    wrist.setPosition(1);
                    Thread.sleep(200);
                    setPathState(5);
                }
                break;
            case 5:
                if (Math.abs(follower.getPose().getX()) > Math.abs((set_Sample3.getX())) - 1 && Math.abs(follower.getPose().getY()) < Math.abs(set_Sample3.getY()) + 1) {
                    follower.setMaxPower(0.8);
                    setMec(1);
                    follower.followPath(keepS4);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Servo_kan(0);
                    wrist.setPosition(1);
                    Thread.sleep(250);
                    neep.setPosition(1);
                    Thread.sleep(200);
                    setMec(0);
                    follower.followPath(Point4_set);
                    setPathState(104);
                }
                break;
            case 104:
                if (!follower.isBusy()) {
                    neep.setPosition(0);
                    Thread.sleep(350);
                    setPathState(1041);
                }
                break;
            case 1041:
                if (!follower.isBusy()) {
                    Servo_kan(0.5);
                    wrist.setPosition(1);

                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setMec(1);
                    follower.followPath(finale);

                    setPathState(777);
                }
                break;
            case 777:
                if (!follower.isBusy()) {
                    P3.setPosition(0.18);
                    Thread.sleep(500);
                }
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
        P3 = hardwareMap.get(Servo.class, "P3");
        wrist.setDirection(Servo.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
    }
}
