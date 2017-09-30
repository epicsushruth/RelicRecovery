package org.firstinspires.ftc.teamcode;

/**
 * Created by Sushruth on 4/21/17.
 */


import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "XTele", group = "X")
public class XTele extends OpMode {

    MecanumHardware robot = new MecanumHardware();
    double lefty;
    double righty;
    double leftx;
    double rightx;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        //driving
        lefty = -gamepad1.left_stick_y;
        righty = gamepad1.right_stick_y;
        leftx = gamepad1.left_stick_x;
        rightx = gamepad1.right_stick_x;
        robot.motorLF.setPower(lefty);
        robot.motorLB.setPower(lefty);
        robot.motorRF.setPower(righty);
        robot.motorRB.setPower(righty);

        while(gamepad1.left_stick_x >= .1 || gamepad1.left_stick_x <= -.1) {
            robot.motorLF.setPower(leftx);
            robot.motorLB.setPower(-leftx);
            robot.motorRF.setPower(leftx);
            robot.motorRB.setPower(-leftx);
        }
    }
}