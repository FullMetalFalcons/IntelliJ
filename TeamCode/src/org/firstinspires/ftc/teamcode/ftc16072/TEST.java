package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import javafx.scene.transform.Rotate;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.ftc16072.Util.Polar;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;

@TeleOp(name = "TEST CODE", group = "FRC 4557")
public class TEST extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;

    private double GetTurnedAngle(double currentHeading, double startHeading) {
        double LeftAngle = 0;
        if (currentHeading < startHeading) {
            LeftAngle = 360 - (startHeading - currentHeading);
        } else {
            LeftAngle = currentHeading - startHeading;
        }
        return LeftAngle;
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        mecanumDrive.init(hardwareMap);
    }

    BNO055IMU.Parameters params = new BNO055IMU.Parameters();

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = 0;
        double strafe = 0;
        double rotate = 0;
        double startHeading = 0;
        double currentHeading = mecanumDrive.getHeading(AngleUnit.DEGREES);
        double turndegree = GetTurnedAngle(currentHeading, startHeading);

        if (gamepad1.left_stick_y * -1 > 0.2) {
            forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed
        }
        if (gamepad1.left_stick_y * -1 < -0.2) {
            forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed
        }
        if (gamepad1.left_stick_x > 0.2) {
            strafe = gamepad1.left_stick_x;
        }
        if (gamepad1.left_stick_x < -0.2) {
            strafe = gamepad1.left_stick_x;
        }
        if (gamepad1.right_stick_x > 0.2) {
            rotate = gamepad1.right_stick_x;
        }
        if (gamepad1.right_stick_x < -0.2) {
            rotate = gamepad1.right_stick_x;
        }


        //mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("rotatedegree", currentHeading);


        boolean leftturn = gamepad1.left_bumper;
        boolean rightturn = gamepad1.right_bumper;
        boolean strafeleft = gamepad1.dpad_left;
        boolean straferight = gamepad1.dpad_right;
        boolean dpadup = gamepad1.dpad_up;
        boolean dpaddown = gamepad1.dpad_down;
        double rotatedegree = mecanumDrive.getHeading(AngleUnit.DEGREES);

        if (leftturn && rightturn) {
            rotate = 0;
        } else if (leftturn && rotatedegree <= 90) {
            rotate = -0.5;
        } else {
            double rightrotatedegree = (turndegree == 0 ? 0 : 360 - turndegree);

            if (rightturn && rightrotatedegree <= 90) {
                rotate = 0.5;
            }
            if (strafeleft) {
                strafe = -1;
            }
            if (straferight) {
                strafe = 1;
            }
            if (strafeleft && straferight) {
                strafe = 0;
            }
            if (dpadup) {
                forward = 1;
            }
            if (dpaddown) {
                forward = -1;
            }
            if (dpadup && dpaddown) {
                forward = 0;
            }
            distances = mecanumDrive.getDistanceCm();
            telemetry.addData("distance fwd", distances[0]);
            telemetry.addData("distance right", distances[1]);
            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);
            telemetry.addData("rotatedegree", mecanumDrive.getHeading(AngleUnit.DEGREES));
            telemetry.addData("leftturn", leftturn);
            telemetry.addData("rightturn", rightturn);
            telemetry.addData("turndegree", GetTurnedAngle(currentHeading, startHeading));
            telemetry.addData("startheading", startHeading);
            telemetry.addData("currentheading", currentHeading);


            mecanumDrive.driveMecanum(forward, strafe, rotate);












/*
        boolean strafeleft = gamepad1.dpad_left;
        boolean straferight = gamepad1.dpad_right;

        if (strafeleft && straferight) {
            mecanumDrive.driveMecanum(0, 0, 0);
        } else {
            if (strafeleft)
            {
                mecanumDrive.driveMecanum(0,-1,0);
            } else {
                if (straferight) {
                    mecanumDrive.driveMecanum(0, 1, 0);
                }
            }
        }



        boolean dpadup = gamepad1.dpad_up;
        boolean dpaddown = gamepad1.dpad_down;

        if (dpadup && dpaddown) {
            forward = 0;
        } else {
            if (dpadup) {
                mecanumDrive.driveMecanum(1,0,0);
            } else {
                if (dpaddown) {
                    mecanumDrive.driveMecanum(-1,0,0);
                }
            }
            }
        }
*/


        }
    }
}

