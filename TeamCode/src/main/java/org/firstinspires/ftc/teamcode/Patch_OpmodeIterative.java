/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

@Disabled
@TeleOp(name="Main OpMode (patch)", group="Patches")
public class Patch_OpmodeIterative extends OpMode {
	// Configurations
	//final int topHandler = 1;
	//final String decFormat = "0.000";
	final String telemetryFloatFormat = "%.3f";
	final double gradualShutdownThreshold = 0.05;

	// Declare members.
	private final ElapsedTime runtime = new ElapsedTime();
	final public DcMotor[] motors = new DcMotor[4];
	final double[] wheelSpeeds = new double[4];
	double fb,lr,turn,extraAxis;


	// Code to run ONCE when the driver hits INIT
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		motors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
		motors[1] = hardwareMap.get(DcMotor.class, "frontRight");
		motors[2] = hardwareMap.get(DcMotor.class, "backLeft");
		motors[3] = hardwareMap.get(DcMotor.class, "backRight");
	}

	//Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

	@Override
	public void init_loop() {}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() { runtime.reset(); }

	@Override
	public void loop() {
		// Setup a variable for each drive wheel to save power level for telemetry

		// - This uses basic math to combine motions and is easier to drive straight.
		fb = -gamepad1.left_stick_y;
		lr = gamepad1.left_stick_x;
		turn  = gamepad1.right_stick_x;
		extraAxis = -gamepad1.right_stick_y;

		mecanumDrive_Cartesian(fb, lr, turn);

		// Display info
		telemetry.addData("Status", "Run Time: " + runtime.toString());
		display("Gpad");
//		telemetry.addLine("Gamepad" + "\r\n")
//				.addData("lefty", telemetryFloatFormat, lr)
//				.addData("lefty", telemetryFloatFormat, fb)
//				.addData("rightx", telemetryFloatFormat, turn)
//				.addData("righty", telemetryFloatFormat, extraAxis);
		telemetry.addLine("Motors" + "\r\n")
				.addData("frontLeft", telemetryFloatFormat, motors[0].getPower())
				.addData("frontRight", telemetryFloatFormat, motors[1].getPower())
				.addData("backtLeft", telemetryFloatFormat, motors[2].getPower())
				.addData("backRight", telemetryFloatFormat, motors[3].getPower());
	}

	public void mecanumDrive_Cartesian(double y,double x,double rotation) {
		wheelSpeeds[0] = y+x+rotation;
		wheelSpeeds[1] = y-x-rotation;
		wheelSpeeds[2] = y-x+rotation;
		wheelSpeeds[3] = y+x-rotation;

		double top = Math.max(Math.abs(wheelSpeeds[0]),
				Math.max(Math.abs(wheelSpeeds[1]),
						Math.max(Math.abs(wheelSpeeds[2]), Math.abs(wheelSpeeds[3]))));

		// 1. Divides wheelSpeeds proportionally
		for (int i = 0; i < wheelSpeeds.length; i++)
			if (wheelSpeeds[i] > 1.0 || wheelSpeeds[i] < -1.0)
				wheelSpeeds[i] = Double.parseDouble(new DecimalFormat("0.000").format(wheelSpeeds[i] / top));
		// 2. Sets all wheelSpeeds above 1 to 1
		//for (int i = 0; i < wheelSpeeds.length; i++)
		//	if ((wheelSpeeds[i])>1.0) wheelSpeeds[i]=1.0;
		//	else if ((wheelSpeeds[i])<-1.0) wheelSpeeds[i]=-1.0;

		// Send calculated power to motors
		motors[0].setPower(-wheelSpeeds[0]);
		motors[1].setPower(wheelSpeeds[1]);
		motors[2].setPower(-wheelSpeeds[2]);
		motors[3].setPower(wheelSpeeds[3]);
	}


	public void display(String x) {
		switch (x) {
			case "Motors":
				telemetry.addLine("Motors" + "\r\n")
						.addData("frontLeft", telemetryFloatFormat, motors[0].getPower())
						.addData("frontRight", telemetryFloatFormat, motors[1].getPower())
						.addData("backtLeft", telemetryFloatFormat, motors[2].getPower())
						.addData("backRight", telemetryFloatFormat, motors[3].getPower());
			case "Gpad":
				telemetry.addLine("Gamepad" + "\r\n")
						.addData("lefty", telemetryFloatFormat, lr)
						.addData("lefty", telemetryFloatFormat, fb)
						.addData("rightx", telemetryFloatFormat, turn)
						.addData("righty", telemetryFloatFormat, extraAxis);
		}
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
		// Slowly turn off the motors after pressing stop
		for (DcMotor motor:motors) {
			/*
			while (motor.getPower()>=gradualShutdownThreshold) {
				motor.setPower(motor.getPower()-gradualShutdownThreshold);
				display("Motors");
			}
			while (motor.getPower()<=-gradualShutdownThreshold) {
				motor.setPower(motor.getPower()+gradualShutdownThreshold);
				display("Motors");
			}
			if (motor.getPower()<=gradualShutdownThreshold*2 && motor.getPower()>=-gradualShutdownThreshold*2) motor.setPower(0); */
			motor.setPower(0.0);
		}
		display("Motors");
		telemetry.addData("Status", "Stopped");
	}
}