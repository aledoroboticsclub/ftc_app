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
		//driving if chain
		double power = 1;
		if(gamepad1.right_bumper)
			power = .5;
		if(gamepad1.left_bumper)
			power=.25;
		if(gamepad1.left_stick_y>.25)
			r.setToForward(power);
		else if(gamepad1.left_stick_y<-.25)
			r.setToBackward(power);
		else if(gamepad1.right_stick_x<-.25)
			r.setToCounterwise(power);
		else if(gamepad1.right_stick_x>.25)
			r.setToClockwise(power);
		else if(gamepad1.left_stick_x>.25)
			r.setToRight(power);
		else if(gamepad1.left_stick_x<-.25)
			r.setToLeft(power);
		else
			r.setToStill();

		//intake servos
		if(gamepad1.a == true) {
			r.setToStartIntake();
		}
		else if(gamepad1.x == true) {
			r.setToReverseIntake();
		}
		else r.stopIntake();
	}
}
