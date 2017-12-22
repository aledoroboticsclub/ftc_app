package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Aledo HS Robotics on 4/12/2017.
 * gamepad1-
 * 			tank drive with joysticks
 * 			strafe with dpad
 *			precise control mode while holding down left bumper
 * gamepad2-
 * 			pulley control with dpad, up for up, down for down
 * 			flicker control with triggers, launch with right
 * 			beaconPusher control with x and b, x for left, b for right
 * 			collector control with a and y, a for take in, y for spit out
 * 			cap ball container control with dpad, left for release, right for holding
 */

@TeleOp(name="TrayServoTester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled     //Determines if the program shows up on Driver Station


public class TrayServoTester extends OpMode
{
	Servo superServo;
	double servoPosition=0;

	DcMotor lift1;
	DcMotor lift2;
    int liftPosition=0;
	public void init()
	{
		superServo=hardwareMap.servo.get("trayServo");
		lift1=hardwareMap.dcMotor.get("lift1");
		lift2=hardwareMap.dcMotor.get("lift2");
		lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void initLoop(){}

	public void start(){}

	public void loop()
	{
		if (gamepad1.a)
		{
			servoPosition+=.001;
		}
		if(gamepad1.x)
		{
			servoPosition-=.001;
		}
		superServo.setPosition(servoPosition);
		telemetry.addData("Servo Position", servoPosition);

		if(gamepad1.right_trigger>.25)
		{
			liftPosition+=10;
		}
		else if(gamepad1.left_trigger>.25)
		{
			liftPosition-=10;
		}
		lift1.setTargetPosition(liftPosition);
		lift2.setTargetPosition(liftPosition);
		lift1.setPower(1);
		lift2.setPower(1);
        telemetry.addData("lift position", liftPosition);
	}
}
