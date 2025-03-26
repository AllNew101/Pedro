package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import java.util.List;
import java.util.concurrent.TransferQueue;
import org.firstinspires.ftc.teamcode.opmode.auto.track;

@Autonomous(name = "Camera_test")
public class S5_Test extends OpMode {


    private track track1 ;
    private List keeper = JavaUtil.createListWith();

    private DcMotor L2 ;
    private DcMotor L1 ;
    private Servo rightservo;
    private Servo leftservo;
    private Servo neep;
    private Servo spin;
    private Servo wrist;


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        wrist = hardwareMap.get(Servo.class, "wrist");
        neep = hardwareMap.get(Servo.class, "neep");
        spin = hardwareMap.get(Servo.class, "spin");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        wrist.setDirection(Servo.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        track1 = new track(hardwareMap,new ColorRange(ColorSpace.HSV, new Scalar(10, 86, 120), new Scalar(84, 255, 255)));

        upread();

    }

    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

        track1.max_area = 6000;
        keeper = track1.track(true);
        telemetry.addData("Read",keeper);



        keeper = track1.track(false);
        telemetry.addData("Read",keeper);

        telemetry.update();


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
    private void Servo_kan(double right) {
        rightservo.setPosition(right);
        leftservo.setPosition(1 - right);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        ;
    }
}
