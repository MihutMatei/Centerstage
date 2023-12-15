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
public class RobotUtilsDebug {

    public ServoImplEx axon_arm_left;
    public ServoImplEx axon_arm_right;
    public ServoImplEx axon_rotire_cuva;
    public Servo pixel_servo; //deschide cuva
//    public DcMotorEx motorIntake;
//    public DcMotorEx sliderLeft;
//    public DcMotorEx sliderRight;

    //-----------------------VALORI SI VARIABILE-------------------------
    public double cuva_score_pos=0;
    public double cuva_return_pos=0;

    public double arm_up_pos=0;
    public double arm_down_pos=0;

    public int slider_high_pos=0;
    public int slider_mid_pos=0;
    public int slider_low_pos=0;
    public int slider_down_pos=0;

    public double pixel_drop_pos=0;
    public double pixel_reset_pos=0;
    public long pixel_delay=250;//ms

    public double intake_on_pow=0;
    public double intake_reverse_pow=0;
    public double intake_off_pow=0;
    public double slider_power=0;


    public RobotUtilsDebug(HardwareMap hardwareMap)
    {
        axon_arm_right=hardwareMap.get(ServoImplEx.class,"ax_brat_dr");
        axon_arm_left=hardwareMap.get(ServoImplEx.class,"ax_brat_st");
        axon_rotire_cuva=hardwareMap.get(ServoImplEx.class,"ax_r_cuva");
        pixel_servo = hardwareMap.get(Servo.class,"srv_cuv");
//        motorIntake = hardwareMap.get(DcMotorEx.class,"m_intake");
//        sliderLeft = hardwareMap.get(DcMotorEx.class, "s_left");
//        sliderRight = hardwareMap.get(DcMotorEx.class, "s_right");

        axon_arm_left.setPwmEnable();
        axon_arm_right.setPwmEnable();
        axon_rotire_cuva.setPwmEnable();

        axon_arm_left.setPwmRange(new PwmControl.PwmRange(505, 2495));
        axon_arm_right.setPwmRange(new PwmControl.PwmRange(505, 2495));
        axon_rotire_cuva.setPwmRange(new PwmControl.PwmRange(505, 2495));
    }


    public void arm_extend(){
        axon_arm_left.setPosition(arm_up_pos);
        axon_arm_right.setPosition(arm_up_pos);
    }
    /**
     * pune cu 2 axoane bratul pe care se afla cuva inapoi in robot, dupa ce a fost apelat cuva_return(), pentru a permite intake_ul
     * */
    public void arm_return(){
        axon_arm_left.setPosition(arm_down_pos);
        axon_arm_left.setPosition(arm_down_pos);
    }

}