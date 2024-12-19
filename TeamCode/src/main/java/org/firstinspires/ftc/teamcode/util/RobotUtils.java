package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Config
public class RobotUtils {

   // public ServoImplEx axon_arm_left;

//    public Servo pixel_servo; //deschide cuva
//    public Servo servo_drona;
//    public DcMotorEx motorIntake;
    public Servo brat_cleste;
    public ServoImplEx brat;
    public ServoImplEx cleste;
    public ServoImplEx bazacleste;

    public ServoImplEx extendo;
    public DcMotorEx sliderLeft;
    public DcMotorEx sliderRight;


    //-----------------------VALORI SI VARIABILE-------------------------


    public static double cleste_open=0.3;
    public static double cleste_close = 0.7;
    public static double brat_jos=0.2;
    public static double brat_sus=0.7;
    public static double brat_cleste_sus=0.4;
    public static double brat_cleste_jos=0.8;
    public static double extendo_extins=0.2;
    public static double extendo_retras=0.7;
    public static double baza_vertical=0.2;
    public static double baza_orizontal =0.7;





    public static int slider_high_pos=3100;//3220 max
    public static int slider_mid_pos=2500;
    public static int slider_low_pos=1600;
    public static int slider_low_auto_pos=900;

    public static int slider_down_pos=35;


    public RobotUtils(HardwareMap hardwareMap)
    {
        brat_cleste=hardwareMap.get(Servo.class,"brat_cleaste");
        brat=hardwareMap.get(ServoImplEx.class,"brat");
       cleste=hardwareMap.get(ServoImplEx.class,"clest");
        bazacleste = hardwareMap.get(ServoImplEx.class,"bazacleste");
        extendo = hardwareMap.get(ServoImplEx.class,"extendo");
        sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotorEx.class, "sliderRight");

    }
    public void setSliderPositions(int position)
    {
        sliderLeft.setTargetPosition(position);
        sliderRight.setTargetPosition(-position);
    }

    public void goSliderToPosition(int position, double power) {

        double absPower = Math.abs(power);

        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int currentPos = sliderLeft.getCurrentPosition();

        // Set the target position of both slider motors.
        setSliderPositions(position);

        // Set the run mode of both slider motors to RUN_TO_POSITION.



        if (currentPos > position) {
            // If the current position is higher than the target position, move the sliders down.
            sliderLeft.setPower(-absPower);
            sliderRight.setPower(absPower);
        }
        else if (currentPos < position) {
            // If the current position is lower than the target position, move the sliders up.
            sliderLeft.setPower(absPower);
            sliderRight.setPower(-absPower);
        }
        // If the current position is already at the target position, the sliders do not need to move.
    }


}