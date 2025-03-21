package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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




    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        track1 = new track(hardwareMap,new ColorRange(ColorSpace.HSV, new Scalar(10, 86, 120), new Scalar(84, 255, 255)));



    }

    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {


        keeper = track1.track();
        track1.max_area = 6000;
        telemetry.addData("Read",keeper);
        telemetry.addData("Read",track1.data_right);
        telemetry.update();

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        ;
    }
}
