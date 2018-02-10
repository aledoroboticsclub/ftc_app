package org.firstinspires.ftc.teamcode.Code2018.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;
//basic drive goes at full power all the time, but has a crawl mode and reverse button
//complete

@TeleOp(name="TeleopDriveBasic", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
     //Determines if the program shows up on Driver Station


public class TeleopDriveBasic extends OpMode
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
				r.setToLeft(power * .75);
			else if (gamepad1.left_stick_x < -.25)
				r.setToRight(power * .75);
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
	}
}

