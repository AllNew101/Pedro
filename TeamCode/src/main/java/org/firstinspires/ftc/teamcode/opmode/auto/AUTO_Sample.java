package org.firstinspires.ftc.teamcode.opmode.auto;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

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

@Autonomous(name = "AutoSample_BLUE")
public class AUTO_Sample extends OpMode {

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
    private track track_color ;
    private List keeper = JavaUtil.createListWith();


    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose set_Sample = new Pose(-18, 4, Math.toRadians(45));
    private final Pose keep_S2 = new Pose(-11, 11.8, Math.toRadians(90));
    private final Pose set_Sample2 = new Pose(-20, 8, Math.toRadians(60));
    private final Pose keep_S3 = new Pose(-20, 11.5, Math.toRadians(90));
    private final Pose set_Sample3 = new Pose(-22, 8, Math.toRadians(50));
    private final Pose keep_S4 = new Pose(-20.8, 13.5, Math.toRadians(112));
    private final Pose set_Sample4 = new Pose(-21.5, 10, Math.toRadians(55));
    private final Pose b_final = new Pose(-10, 55, Math.toRadians(55));
    private Pose final0 = new Pose(12.5,55.72,Math.toRadians(0));
    private Pose keep_S5 = new Pose(12.5,51.72,Math.toRadians(0));
    private final Pose set_Sample5 = new Pose(-24, 8, Math.toRadians(45));
    private  Pose final1 = new Pose(7.28,55.72,Math.toRadians(0));



    private Path Point1_set;
    private PathChain keepS2, keepS3, Point2_set,  Point3_set, keepS4,Point4_set,finale,keep5,set_Sample_5,keep6,keepSp3,hangSp3,hangSp4,hangSp5,keepSp1,keepSp2,Finish;

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
        keep5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(final0), new Point(keep_S5)))
                .setLinearHeadingInterpolation(final0.getHeading(), keep_S5.getHeading())
                .build();
        set_Sample_5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_S5),new Point(b_final), new Point(set_Sample5)))
                .setLinearHeadingInterpolation(keep_S5.getHeading(), set_Sample5.getHeading())
                .build();
        keep6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(set_Sample5),new Point(b_final), new Point(final1)))
                .setLinearHeadingInterpolation(set_Sample5.getHeading(), final1.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.9);
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
                    follower.setMaxPower(0.6);
                    setMec(1);
                    follower.followPath(keepS2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
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
                    Thread.sleep(700);
                    setPathState(1021);
                }
                break;
            case 1021:
                if (!follower.isBusy()) {
                    Servo_kan(0.12);
                    wrist.setPosition(1);
                    setPathState(3);
                }
                break;
            case 3:
                if (Math.abs(follower.getPose().getX()) > Math.abs((set_Sample2.getX())) - 0.25 && Math.abs(follower.getPose().getY()) < Math.abs(set_Sample2.getY()) + 0.25) {
                    follower.setMaxPower(0.5);
                    setMec(1);
                    follower.followPath(keepS3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
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
                    Thread.sleep(850);
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
                if (Math.abs(follower.getPose().getX()) > Math.abs((set_Sample3.getX())) - 0.25 && Math.abs(follower.getPose().getY()) < Math.abs(set_Sample3.getY()) + 0.25) {
                    follower.setMaxPower(0.6);
                    setMec(1);
                    follower.followPath(keepS4);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    Servo_kan(0);
                    wrist.setPosition(1);
                    Thread.sleep(500);
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
                    Thread.sleep(700);
                    setPathState(1041);
                }
                break;
            case 1041:
                if (!follower.isBusy()) {
                    Servo_kan(0.1);
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
                    P3.setPosition(0.13);
                    setPathState(80);
                }
            case 80:
                if (!follower.isBusy()) {
                    upread();
                    final0 = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0));
                    follower.setMaxPower(0.8);

                    track_color.min_area = 2000;
                    keeper = track_color.track();
                    telemetry.addData("Read",keeper);
                    telemetry.addData("Read",track_color.data_right);
                    spin.setPosition(0);
                    if ((Integer) keeper.get(1) == 1){
                        setPathState(802);
                    } else{
                        keep_S5 = new Pose(follower.getPose().getX() + (double)keeper.get(3),follower.getPose().getY() + (double)keeper.get(2),0);
                        keep5 = follower.pathBuilder()
                                .addPath(new BezierLine(new Point(final0), new Point(keep_S5)))
                                .setLinearHeadingInterpolation(final0.getHeading(), keep_S5.getHeading())
                                .build();
                        follower.followPath(keep5);
                        if ((Integer) keeper.get(1) == 2){
                            setPathState(802);
                        }
                        else{
                            keep_S5 = new Pose(follower.getPose().getX(),follower.getPose().getY() + 3,0);
                            keep5 = follower.pathBuilder()
                                    .addPath(new BezierLine(new Point(final0), new Point(keep_S5)))
                                    .setLinearHeadingInterpolation(final0.getHeading(), keep_S5.getHeading())
                                    .build();
                            follower.followPath(keep5);
                            setPathState(80);
                        }
                    }

                }
                break;
            case 802:
                if (!follower.isBusy()) {
                    downkeep();
                    setPathState(8);

                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(set_Sample_5);
                    setPathState(201);
                }
                break;
            case 201:
                if (Math.abs(follower.getPose().getX()) > Math.abs(set_Sample5.getX()) - 16 && follower.getPose().getY() < (set_Sample5.getY() + 30)) {
                    setMec(0);
                    setPathState(202);
                }
                break;
            case 202:
                if (!follower.isBusy()) {
                    Thread.sleep(200);
                    neep.setPosition(0);
                    Thread.sleep(300);
                    Servo_kan(0.15);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    Servo_kan(0.15);
                    wrist.setPosition(1);
                    follower.followPath(keep6);
                    setMec(1);
                    setPathState(80);
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
        track_color = new track(hardwareMap, ColorRange.YELLOW);
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
    private void upread(){
        while (true) {
            if (L2.getCurrentPosition() <= 400) {
                L1.setPower(0.6);
                L2.setPower(0.6);
                wrist.setPosition(0.8);
                neep.setPosition(0.2);
                Servo_kan(0);
            } else {
                L1.setPower(0.05);
                L2.setPower(0.05);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            }}
    }
    private void downkeep() throws InterruptedException{
        wrist.setPosition(1);
        while (true) {
            if (Math.abs(L2.getCurrentPosition()) >= 15) {
                L1.setPower(-0.3);
                L2.setPower(-0.3);
            } else {
                L1.setPower(0);
                L2.setPower(0);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            }
        }
        Thread.sleep(250);
        neep.setPosition(1);
        Thread.sleep(250);
        Servo_kan(0.15);
        Thread.sleep(100);
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
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
