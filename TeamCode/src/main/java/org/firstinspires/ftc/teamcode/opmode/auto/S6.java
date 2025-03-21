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

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
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

    private DcMotor L2;
    private DcMotor L1;
    private long time2;
    boolean check = false;
    boolean ratio = false;
    double angle = 0.0;
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
    private final Pose plan_1 = new Pose(0, 1.5, Math.toRadians(0));
    private final Pose plan_2 = new Pose(0, 3, Math.toRadians(0));
    private final Pose plan_3 = new Pose(0, 4.5, Math.toRadians(0));
    private final Pose plan_4 = new Pose(0, 6, Math.toRadians(0));
    private final Pose plan_5 = new Pose(0, 7.5, Math.toRadians(0));
    private final Pose plan1_1 = new Pose(2, 1.5, Math.toRadians(0));
    private final Pose plan2_1 = new Pose(2, 3, Math.toRadians(0));
    private final Pose plan3_1 = new Pose(2, 4.5, Math.toRadians(0));
    private final Pose plan4_1 = new Pose(2, 6, Math.toRadians(0));
    private final Pose plan5_1 = new Pose(2, 7.5, Math.toRadians(0));
    private final Pose plan1_2 = new Pose(4, 1.5, Math.toRadians(0));
    private final Pose plan2_2 = new Pose(4, 3, Math.toRadians(0));
    private final Pose plan3_2 = new Pose(4, 4.5, Math.toRadians(0));
    private final Pose plan4_2 = new Pose(4, 6, Math.toRadians(0));
    private final Pose plan5_2 = new Pose(4, 7.5, Math.toRadians(0));
    private final Pose plan1_3 = new Pose(-2, 1.5, Math.toRadians(0));
    private final Pose plan2_3 = new Pose(-2, 3, Math.toRadians(0));
    private final Pose plan3_3 = new Pose(-2, 4.5, Math.toRadians(0));
    private final Pose plan4_3 = new Pose(-2, 6, Math.toRadians(0));
    private final Pose plan5_3 = new Pose(-2, 7.5, Math.toRadians(0));


    private Path Point1_set;
    private PathChain keep50, keep51, keep52, keep53, keep54, keep60, keep61, keep62, keep63, keep64, keep65, keep66, keep67, keep68, keep69, keepSp2, Finish, keep5;

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
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
        keep60 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan1_1), new Point(plan2_1)))
                .setLinearHeadingInterpolation(plan1_1.getHeading(), plan2_1.getHeading())
                .build();
        keep61 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan2_1), new Point(plan3_1)))
                .setLinearHeadingInterpolation(plan2_1.getHeading(), plan3_1.getHeading())
                .build();
        keep62 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan3_1), new Point(plan4_1)))
                .setLinearHeadingInterpolation(plan3_1.getHeading(), plan4_1.getHeading())
                .build();
        keep63 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan4_1), new Point(plan5_1)))
                .setLinearHeadingInterpolation(plan4_1.getHeading(), plan5_1.getHeading())
                .build();
        keep64 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan5_1), new Point(plan1_2)))
                .setLinearHeadingInterpolation(plan5_1.getHeading(), plan1_2.getHeading())
                .build();
        keep65 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan1_2), new Point(plan2_2)))
                .setLinearHeadingInterpolation(plan1_2.getHeading(), plan2_2.getHeading())
                .build();
        keep66 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan2_2), new Point(plan3_2)))
                .setLinearHeadingInterpolation(plan2_2.getHeading(), plan3_2.getHeading())
                .build();
        keep67 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan3_2), new Point(plan4_2)))
                .setLinearHeadingInterpolation(plan3_2.getHeading(), plan4_2.getHeading())
                .build();
        keep68 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan4_2), new Point(plan5_2)))
                .setLinearHeadingInterpolation(plan4_2.getHeading(), plan5_2.getHeading())
                .build();
        keep69 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(plan5_2), new Point(plan1_3)))
                .setLinearHeadingInterpolation(plan5_2.getHeading(), plan1_3.getHeading())
                .build();

    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                while (true) {
                    if (L2.getCurrentPosition() <= 400) {
                        L1.setPower(0.6);
                        L2.setPower(0.6);
                        wrist.setPosition(0.8);
                        neep.setPosition(0.28);
                        Servo_kan(0);
                    } else {
                        L1.setPower(0.05);
                        L2.setPower(0.05);
                        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        break;
                    }}
            case 1:
                follower.setMaxPower(1);
                track();
                if (check == false) {
                    setPathState(808);
                } else if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(keep50);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep51);
                        setPathState(3);
                }}
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep52);
                        setPathState(4);
                    }}
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep53);
                        setPathState(5);
                    }}
                break;
            case 5:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep54);
                        setPathState(6);
                    }}
                break;
            case 6:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep60);
                        setPathState(7);
                    }}
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep61);
                        setPathState(8);
                    }}
                break;
            case 8:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep62);
                        setPathState(9);
                    }}
                break;
            case 9:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep63);
                        setPathState(10);
                    }}
                break;
            case 10:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep64);
                        setPathState(11);
                    }}
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep65);
                        setPathState(12);
                    }}
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep66);
                        setPathState(13);
                    }}
                break;
            case 13:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep67);
                        setPathState(14);

                    }}
                break;
            case 14:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    track();
                    if (check == false) {
                        setPathState(808);
                    } else if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        follower.followPath(keep68);
                        setPathState(909);
                    }}
                break;
            case 909:
                if (check == false) {
                    setPathState(-1);
                } else if (!follower.isBusy()) {
                    P3.setPosition(0.18);
                    Thread.sleep(1000);
                    setPathState(-1);

                }
                break;
            case 808:
                wrist.setPosition(1);
                Thread.sleep(500);
                while (true) {
                    if (Math.abs(L2.getCurrentPosition()) >= 15) {
                        L1.setPower(-0.6);
                        L2.setPower(-0.6);
                    } else {
                        L1.setPower(0);
                        L2.setPower(0);
                        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        break;
                    }
                }
                Thread.sleep(700);
                neep.setPosition(1);
                Thread.sleep(1000);
                setPathState(-1);
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

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setMec(int pMec) {
        mec = pMec;
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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
        telemetry.addData("lift1", L1.getCurrentPosition());
        telemetry.addData("lift2", L2.getCurrentPosition());
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
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


        // Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
        myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
        // - Specify the color range you are looking for.
        // - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
        //     This can be the entire frame, or a sub-region defined using:
        //     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
        //     Use one form of the ImageRegion class to define the ROI.
        myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.YELLOW);
        // 50% width/height square centered on screen
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0, 0.5, -1));
        // - Define which contours are included.
        //     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
        //     note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        // - Turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
        myColorBlobLocatorProcessorBuilder.setDrawContours(true);
        // - Include any pre-processing of the image or mask before looking for Blobs.
        //     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
        //     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
        //     Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
        //     The higher the number of pixels, the more blurred the image becomes.
        //     Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
        //     Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
        myColorBlobLocatorProcessorBuilder.setBlurSize(5);
        //     Erosion removes floating pixels and thin lines so that only substantive objects remain.
        //     Erosion can grow holes inside regions, and also shrink objects.
        //     "pixels" in the range of 2-4 are suitable for low res images.
        myColorBlobLocatorProcessorBuilder.setErodeSize(5);
        //     Dilation makes objects more visible by filling in small holes, making lines appear thicker,
        //     and making filled shapes appear larger. Dilation is useful for joining broken parts of an
        //     object, such as when removing noise from an image.
        //     "pixels" in the range of 2-4 are suitable for low res images.
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
        // Speed up telemetry updates, Just use for debugging.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setMec(-1);

    }

    public void track() throws InterruptedException {

        check = true;
        // Read the current list of blobs.
        myBlobs = myColorBlobLocatorProcessor.getBlobs();
        // The list of Blobs can be filtered to remove unwanted Blobs.
        //   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
        //             conditions will remain in the current list of "blobs".  Multiple filters may be used.
        //
        // Use any of the following filters.
        //
        // A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
        // Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
        ColorBlobLocatorProcessor.Util.filterByArea(2000, 20000, myBlobs);
        // A blob's density is an indication of how "full" the contour is.
        // If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
        // The density is the ratio of Contour-area to Convex Hull-area.
        // A blob's Aspect ratio is the ratio of boxFit long side to short side.
        // A perfect Square has an aspect ratio of 1.  All others are > 1
        // The list of Blobs can be sorted using the same Blob attributes as listed above.
        // No more than one sort call should be made.  Sorting can use ascending or descending order.
        // Display the size (area) and center location for each Blob.
        for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
            myBlob = myBlob_item;
            // Get a "best-fit" bounding box (called "boxFit", of type RotatedRect) for this blob.
            myBoxFit = myBlob.getBoxFit();
            mySize = myBoxFit.boundingRect();
            if (mySize.width / mySize.height > 1) {
                ratio = true;
                angle = myBoxFit.angle;
            } else if (mySize.width / mySize.height < 1) {
                ratio = false;
                angle = myBoxFit.angle + 360;
            }
            telemetry.addData("key", angle);
            if (check) {
                check = false;
                if (angle >= 360) {
                    spin.setPosition(0);
                    if (Math.abs(160 - myBoxFit.center.x) <= 10){
                        check = false;
                    }
                    else {
                        check = true;
                    }
                } else {
                    spin.setPosition(0.5);
                    if (Math.abs(160 - myBoxFit.center.x) <= 28){
                        check = false;
                    }
                    else {
                        check = true;
                    }
                }

            }
        }
        Thread.sleep(500);
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
