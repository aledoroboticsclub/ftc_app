package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleopStrider", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
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
		if (gamepad1.right_bumper)
			power = .5;
		if (gamepad1.left_bumper)
			power = .25;
		if (gamepad1.left_stick_y > .25)
			r.setToForward(power);
		else if (gamepad1.left_stick_y < -.25)
			r.setToBackward(power);
		else if (gamepad1.right_stick_x < -.25)
			r.setToCounterwise(power);
		else if (gamepad1.right_stick_x > .25)
			r.setToClockwise(power);
		else if (gamepad1.left_stick_x > .25)
			r.setToRight(power);
		else if (gamepad1.left_stick_x < -.25)
			r.setToLeft(power);
		else
			r.setToStill();

		//intake motors and servos
		if(gamepad1.a == true)
			r.setToStartIntake(power);
		else if(gamepad1.x == true)
			r.setToReverseIntake();
		else r.stopIntake();

		//tray
		if(gamepad1.b)
			r.setTrayToIntake();
		else if(gamepad1.y)
			r.setTrayToPlace();

		/*//lift
		if(gamepad1.right_trigger>.1)
			r.setLiftToUp(gamepad1.right_trigger);
		else if(gamepad1.left_trigger>.1)
			r.setLiftToDown(gamepad1.left_trigger);
		else
			r.setLiftToStill();*/

		//lift
		if (gamepad1.left_stick_button)
			r.setLiftToPosition0();
		else if (gamepad1.dpad_right)
			r.setLiftToPosition1();
		else if (gamepad1.dpad_left)
			r.setLiftToPosition2();
		else if (gamepad1.dpad_up)
			r.setLiftToPosition3();
		else if (gamepad1.dpad_down)
			r.setLiftToPosition4();

		telemetry.update();
	}
}
