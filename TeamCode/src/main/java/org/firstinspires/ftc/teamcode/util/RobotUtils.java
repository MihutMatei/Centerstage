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

    public ServoImplEx axon_arm_left;
    public ServoImplEx axon_arm_right;
    public ServoImplEx axon_rotire_cuva;
    public Servo pixel_servo; //deschide cuva
    public Servo servo_drona;
    public DcMotorEx motorIntake;
    public DcMotorEx sliderLeft;
    public DcMotorEx sliderRight;

    //-----------------------VALORI SI VARIABILE-------------------------
    public static double cuva_score_pos=0.85;
    public static double cuva_return_pos=0.46;
//    public static double cuva_clearance_pos=0.4;

    public static double arm_up_pos=0.82;
    public static double arm_down_pos=0.376;
//    public static double arm_clearance_pos =0.4;

    public static double drone_reset_pos=0;
    public static double launch_pos=0.2;

    public static int slider_high_pos=3100;//3220 max
    public static int slider_mid_pos=2500;
    public static int slider_low_pos=1300;
    public static int slider_low_auto_pos=900;

    public static int slider_down_pos=35;

    public static double pixel_drop_pos=0.59;
    public static double pixel_reset_pos=0.485;
    public static long pixel_delay=400;//ms

    public static double intake_on_pow=0.75;
    public static double intake_reverse_pow=-0.75;
    public static double intake_off_pow=0;
    public static double slider_power=0.8;


    public RobotUtils(HardwareMap hardwareMap)
    {
        axon_arm_right=hardwareMap.get(ServoImplEx.class,"ax_brat_dr");
        axon_arm_left=hardwareMap.get(ServoImplEx.class,"ax_brat_st");
        axon_rotire_cuva=hardwareMap.get(ServoImplEx.class,"ax_r_cuva");
        pixel_servo = hardwareMap.get(Servo.class,"srv_cuv");
        motorIntake = hardwareMap.get(DcMotorEx.class,"m_intake");
        sliderLeft = hardwareMap.get(DcMotorEx.class, "s_left");
        sliderRight = hardwareMap.get(DcMotorEx.class, "s_right");
        servo_drona = hardwareMap.get(Servo.class,"drn");
        axon_arm_left.setPwmEnable();
        axon_arm_right.setPwmEnable();
        axon_rotire_cuva.setPwmEnable();

        axon_arm_left.setPwmRange(new PwmControl.PwmRange(505, 2495));
        axon_arm_right.setPwmRange(new PwmControl.PwmRange(505, 2495));
        axon_rotire_cuva.setPwmRange(new PwmControl.PwmRange(500, 2500));
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

    /**
     * pune cu un srvo cuva in locul in care se da score
     * */
    public void cuva_score(){
        axon_rotire_cuva.setPosition(cuva_score_pos);
    }
    /**
     * pune cuva in pozitia de clearance
     * */
//    public void cuva_clearance(){
//        axon_rotire_cuva.setPosition(cuva_clearance_pos);
//    }
    /**
     * pune cuva in pozitia in care trebuie sa fie pentru intake
     * */
    public void cuva_return(){
        axon_rotire_cuva.setPosition(cuva_return_pos);
    }
    /**
     * drop la un singur pixel, servo se duce la pozitia: x si dupa se intoarce ca sa tina al doile pixel daca exista
     * */
    public void pixel_drop_one() {
        pixel_servo.setPosition(pixel_drop_pos);

        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        Runnable task = () -> {
            try {
                pixel_servo.setPosition(pixel_reset_pos);
            } catch (Exception e) {
                telemetry.addData(e.getMessage()," am belit pula");
            } finally {
                // Shutdown the executor after the task is complete
                executor.shutdown();
            }
        };

        // Schedule the task and shutdown the executor gracefully after completion
        executor.schedule(task, pixel_delay, TimeUnit.MILLISECONDS);
    }
    /**
     * drop la toate pixels
     * */

    public void pixel_drop_all(){
        pixel_servo.setPosition(pixel_drop_pos);
    }
    /**
     * ridica servoul care tine in loc pixelii pentru a permite intake-ul
     * */
    public void pixel_reset(){
        pixel_servo.setPosition(pixel_reset_pos);
    }
    /**
     * pune cu 2 axoane bratul pe care se afla cuva in fata robotului pentru a apela dupa cuva_score()
     * */
    public void arm_extend(){
        axon_arm_left.setPosition(arm_up_pos);
        axon_arm_right.setPosition(arm_up_pos);
    }
    /**
     * pune cu 2 axoane bratul pe care se afla cuva in pozitia de clearance
     * */
//    public void arm_clearance(){
//        axon_arm_left.setPosition(arm_clearance_pos);
//        axon_arm_right.setPosition(arm_clearance_pos);
//    }
    /**
     * pune cu 2 axoane bratul pe care se afla cuva inapoi in robot, dupa ce a fost apelat cuva_return(), pentru a permite intake_ul
     * */
    public void arm_return(){
        axon_arm_left.setPosition(arm_down_pos);
        axon_arm_right.setPosition(arm_down_pos);
    }

    public void go_sliders_high(){
        goSliderToPosition(slider_high_pos,slider_power);
    }
    public void go_sliders_mid(){
        goSliderToPosition(slider_mid_pos,slider_power);
    }
    public void go_sliders_low(){
        goSliderToPosition(slider_low_pos,slider_power);
    }
    public void go_sliders_low_auto(){
        goSliderToPosition(slider_low_auto_pos,slider_power);
    }
    public void go_sliders_down(){
        goSliderToPosition(slider_down_pos,slider_power);
    }
    /**
     * porneste motor intake
     * */
    public void intake_on(){
        motorIntake.setPower(intake_on_pow);
    }
    /**
     * opreste motor intake
     * */
    public void intake_off(){
        motorIntake.setPower(intake_off_pow);
    }
    /**
     * porneste motor intake invers
     * */
    public void intake_reverse(){
        motorIntake.setPower(intake_reverse_pow);
    }

    /**
     * lanseaza drona ridicand un servo
     * */
    public void drone_launch(){
        servo_drona.setPosition(launch_pos);
    }
    /**
     * reseteaza servoul utilizat pentru lansarea dronei
     * */
    public void drone_reset(){
        servo_drona.setPosition(drone_reset_pos);
    }
}