package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopScorpion", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled     //Determines if the program shows up on Driver Station


public class TeleopScorpion extends OpMode
{
	Scorpion r=new Scorpion();
	ElapsedTime runtime=new ElapsedTime();
	public void init() {
		r.initRobot(hardwareMap,telemetry);
		telemetry.addData("init Complete","");

		telemetry.update();
	}

	public void initLoop(){}

	public void start(){
		runtime.reset();
	}

	boolean inReverse=false;//reverse button is a
	boolean aWasPressed=false;
	public void loop() {
		//TODO: see if you can change the strafing to remain straight while strafing, maybe use a gyro
		if(gamepad1.a && !aWasPressed)
			inReverse=!inReverse;
		aWasPressed=gamepad1.a;
		//driving if chain
		double power=1;
		if (gamepad1.right_bumper)
			power = .5;
		if (gamepad1.left_bumper)
			power = .25;
		if(inReverse) {//reversed, for placing
			if (gamepad1.left_stick_y > .25)
				r.setToBackward(power);
			else if (gamepad1.left_stick_y < -.25)
				r.setToForward(power);
			else if (gamepad1.right_stick_x < -.25)
				r.setToCounterwise(power);
			else if (gamepad1.right_stick_x > .25)
				r.setToClockwise(power);
			else if (gamepad1.left_stick_x > .25)
				r.setToLeft(power*.75);
			else if (gamepad1.left_stick_x < -.25)
				r.setToRight(power*.75);
			else
				r.setToStill();
		}
		else{//normal running, for picking up
			if (gamepad1.left_stick_y > .25)
				r.setToForward(power);
			else if (gamepad1.left_stick_y < -.25)
				r.setToBackward(power);
			else if (gamepad1.right_stick_x < -.25)
				r.setToCounterwise(power);
			else if (gamepad1.right_stick_x > .25)
				r.setToClockwise(power);
			else if (gamepad1.left_stick_x > .25)
				r.setToRight(power*.75);
			else if (gamepad1.left_stick_x < -.25)
				r.setToLeft(power*.75);
			else
				r.setToStill();
		}


		//tray
		if(gamepad2.a)
			r.setTrayToIntake();
		else if(gamepad2.y)
			r.setTrayToPlace();
		else if(gamepad2.x)
			r.setTrayToParallel();

		//intake
		if(gamepad2.right_trigger>.1)
			r.rightIntake(gamepad2.right_trigger);
		else if(gamepad2.right_bumper)
			r.rightIntake(gamepad1.b?-1:-.5);
		else
			r.rightIntake(0);
		if (gamepad2.left_trigger>.1)
			r.leftIntake(gamepad2.left_trigger);
		else if(gamepad2.left_bumper)
			r.leftIntake(gamepad1.b?-1:-.5);
		else
			r.leftIntake(0);

		//lift
		if(gamepad2.left_stick_y>.1)
			r.moveLiftUp(gamepad2.left_stick_y);
		else if(gamepad2.left_stick_y<-.1)
			r.moveLiftDown(gamepad2.left_stick_y*-1);

		//lift
		if (gamepad2.dpad_down)
			r.setLiftToPosition0();//placing over ground and loading
		else if (gamepad2.dpad_left)
			r.setLiftToPosition1();//placing over 1 cube
		else if (gamepad2.dpad_right)
			r.setLiftToPosition2();//placing over 2 cubes
		else if (gamepad2.dpad_up)
			r.setLiftToPosition3();//placing over 3 cubes

		//relic
		if (gamepad2.right_stick_x<-.75)
			r.setGrabberToGrabbed();
		else if (gamepad2.right_stick_x>.75)
			r.setGrabberToRelease();
		telemetry.addData("gamepad2.right_stick_x",gamepad2.right_stick_x);

		//extender
		if(gamepad2.right_stick_y>.25)
			r.extenderOut();
		else if(gamepad2.right_stick_y<-.25)
			r.extenderIn();
		else
			r.extenderStill();
		telemetry.addData("Runtime: ", runtime.seconds());
		telemetry.update();
	}
}
