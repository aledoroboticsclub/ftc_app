package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



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

@TeleOp(name="TeleopStrider ", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled     //Determines if the program shows up on Driver Station


public class TeleopStrider extends OpMode
{
	Strider r=new Strider();
	public void init()
	{
		r.initRobot(hardwareMap,telemetry);
		telemetry.addData("init Complete","");
		telemetry.update();
	}

	public void initLoop(){}

	public void start(){}

	public void loop()
	{
		double driveSpeed = 1;
		if(gamepad1.right_bumper)
		{
			driveSpeed = .5;
		}
		if(gamepad1.left_stick_y>.25)
		{
			r.setToForward(driveSpeed);
		}
		else if(gamepad1.left_stick_y<-.25)
		{
			r.setToBackward(driveSpeed);
		}
		else if(gamepad1.right_stick_x<-.25)
		{
			r.setToCounterwise(driveSpeed);
		}
		else if(gamepad1.right_stick_x>.25)
		{
			r.setToClockwise(driveSpeed);
		}
		else if(gamepad1.left_stick_x>.25)
		{
			r.setToRight(driveSpeed);
		}
		else if(gamepad1.left_stick_x<-.25)
		{
			r.setToLeft(driveSpeed);
		}
		else
		{
			r.setToStill();
		}

//		if(gamepad1.a == true)
//		{
//			r.runIntake();
//		}
//		else if(gamepad1.a == false)
//		{
//			r.stopIntake();
//		}
	}
}
