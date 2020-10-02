package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import virtual_robot.util.AngleUtils;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Example OpMode. Controls robot using left joystick, with arcade drive.
 */

//@Disabled
@Autonomous(name = "Auto", group = "Mechanum")
public class Auto extends LinearOpMode {
    public void runOpMode() {
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            m1.setPower(1);
            m2.setPower(1);
            m3.setPower(1);
            m4.setPower(1);
            Thread.sleep(1000);
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(1);
            m4.setPower(1);
            Thread.sleep(900);
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(-1);
            Thread.sleep(1500);
            m1.setPower(-1);
            m2.setPower(1);
            m3.setPower(-1);
            m4.setPower(1);
            Thread.sleep(700);
        } catch(InterruptedException e){
            System.out.println("Interrupted");
        }


    }

    public void DriveForward(double power) {

    }
}
