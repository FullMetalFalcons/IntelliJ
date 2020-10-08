package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    protected DcMotor m1 = null;
    protected DcMotor m2 = null;
    protected DcMotor m3 = null;
    protected DcMotor m4 = null;
    public void RobotStop()
    {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
    public int InchesToTick(double inches) {
        double revValue = 1120/inchesPerRev * inches;
        return (int)Math.round(revValue);
    }
    public void resetDriveUsingEncoder() {
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private final double inchesPerRev = Math.PI * 2.0;
    public void DriveToTick(int tick, double power) throws Exception {


        m1.setTargetPosition(tick);
        m2.setTargetPosition(tick);
        m3.setTargetPosition(tick);
        m4.setTargetPosition(tick);

        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);


        Thread.sleep(10);
        boolean notInPosition = true;
        do {
            int m1Pos = m1.getCurrentPosition();
            int m2Pos = m2.getCurrentPosition();
            int m3Pos = m3.getCurrentPosition();
            int m4Pos = m4.getCurrentPosition();
            int avgPos = (m1Pos + m2Pos + m3Pos + m4Pos)/4;
//            Log.d("Avg Tic:",Integer.toString(avgPos));
//            Log.d("DriveToTick:m1",Integer.toString(m1Pos));
//            Log.d("DriveToTick:m2",Integer.toString(m2Pos ));
//            Log.d("DriveToTick:m3",Integer.toString(m3Pos));
//            Log.d("DriveToTick:m4",Integer.toString(m4Pos ));
//            Log.d("NotInPosition", Boolean.toString(notInPosition));

            notInPosition = Math.abs(tick-avgPos) > 30;
            Thread.sleep(20);

        } while (notInPosition);



        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    };
    public void StrafeToTick(int tick, double power) throws Exception {


        m1.setTargetPosition(tick);
        m2.setTargetPosition(tick);
        m3.setTargetPosition(tick);
        m4.setTargetPosition(tick);

        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m1.setPower(power);
        m2.setPower(-power);
        m3.setPower(power);
        m4.setPower(-power);


        Thread.sleep(10);
        boolean notInPosition = true;
        do {
            int m1Pos = m1.getCurrentPosition();
            int m2Pos = m2.getCurrentPosition();
            int m3Pos = m3.getCurrentPosition();
            int m4Pos = m4.getCurrentPosition();
            int avgPos = (m1Pos + m2Pos + m3Pos + m4Pos)/4;
//            Log.d("Avg Tic:",Integer.toString(avgPos));
//            Log.d("DriveToTick:m1",Integer.toString(m1Pos));
//            Log.d("DriveToTick:m2",Integer.toString(m2Pos ));
//            Log.d("DriveToTick:m3",Integer.toString(m3Pos));
//            Log.d("DriveToTick:m4",Integer.toString(m4Pos ));
//            Log.d("NotInPosition", Boolean.toString(notInPosition));

            notInPosition = Math.abs(tick-avgPos) > 30;
            Thread.sleep(20);

        } while (notInPosition);



        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    };
    public void DebugPrint (Telemetry t)
    {
        t.addData("m1", m1.getCurrentPosition());
        t.addData("m2", m2.getCurrentPosition());
        t.addData("m3", m3.getCurrentPosition());
        t.addData("m4", m4.getCurrentPosition());

    }
}


