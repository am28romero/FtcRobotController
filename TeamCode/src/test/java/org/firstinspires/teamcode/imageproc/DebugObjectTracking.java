/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.teamcode.imageproc;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Object Tracking (debug)", group="Debug")
public class DebugObjectTracking extends LinearOpMode {
	OpenCvCamera phoneCam;
	final Size cameraRes = new Size(640, 480);
	final int cameraType = 0;
	Pipeline pipeline = new Pipeline();

	@Override
	public void runOpMode() {
		// Make an instance of the phone camera
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				// Start Streaming
				phoneCam.startStreaming((int) cameraRes.width, (int) cameraRes.height, OpenCvCameraRotation.UPRIGHT);
			}
		});

		telemetry.addLine("Setup Finished Successfully. Waiting for Start...");
		Log.i("Status","Setup Finished Successfully. Waiting for Start...");
		telemetry.update();

		waitForStart();
		resetStartTime();
		telemetry.addLine("Starting...");
		phoneCam.setPipeline(new Pipeline());

		int mode=0;
		while (opModeIsActive()) {
			if (gamepad1.dpad_left || gamepad1.dpad_right) {
				if (gamepad1.dpad_left)mode=mode+1;
				else if (gamepad1.dpad_right) mode=mode+2;
				if (mode>5) mode=mode-5;
			}

			if(gamepad1.dpad_up) {
				switch (mode) {
					case 0: pipeline.minValues.set(new double[]{pipeline.minValues.val[0]+2,pipeline.minValues.val[1],pipeline.minValues.val[2]});
					case 1: pipeline.maxValues.set(new double[]{pipeline.maxValues.val[0]+2,pipeline.maxValues.val[1],pipeline.maxValues.val[2]});
					case 2: pipeline.minValues.set(new double[]{pipeline.minValues.val[0],pipeline.minValues.val[1]+2,pipeline.minValues.val[2]});
					case 3: pipeline.maxValues.set(new double[]{pipeline.maxValues.val[0],pipeline.maxValues.val[1]+2,pipeline.maxValues.val[2]});
					case 4: pipeline.minValues.set(new double[]{pipeline.minValues.val[0],pipeline.minValues.val[1],pipeline.minValues.val[2]+2});
					case 5: pipeline.maxValues.set(new double[]{pipeline.maxValues.val[0],pipeline.maxValues.val[1],pipeline.maxValues.val[2]+2});
				}
			} else if (gamepad1.dpad_down) {
				switch (mode) {
					case 0: pipeline.minValues.set(new double[]{pipeline.minValues.val[0]-2,pipeline.minValues.val[1],pipeline.minValues.val[2]});
					case 1: pipeline.maxValues.set(new double[]{pipeline.maxValues.val[0]-2,pipeline.maxValues.val[1],pipeline.maxValues.val[2]});
					case 2: pipeline.minValues.set(new double[]{pipeline.minValues.val[0],pipeline.minValues.val[1]-2,pipeline.minValues.val[2]});
					case 3: pipeline.maxValues.set(new double[]{pipeline.maxValues.val[0],pipeline.maxValues.val[1]-2,pipeline.maxValues.val[2]});
					case 4: pipeline.minValues.set(new double[]{pipeline.minValues.val[0],pipeline.minValues.val[1],pipeline.minValues.val[2]-2});
					case 5: pipeline.maxValues.set(new double[]{pipeline.maxValues.val[0],pipeline.maxValues.val[1],pipeline.maxValues.val[2]-2});
				}
			}

			if(gamepad1.a) {
				phoneCam.stopStreaming();
			}
			telemetry.addData("Quick Edit Mode",mode==0?"Max Values":"Min Values");
			telemetry.addData("Min Values",pipeline.minValues.toString());
			telemetry.addData("Max Values",pipeline.maxValues.toString());
			sleep(100);
		}
	}

	class Pipeline extends OpenCvPipeline {
		boolean viewportPaused = false;

		// H ranges 0-180, S and V range 0-255
		Scalar maxValues = new Scalar(40, 255, 255);
		Scalar minValues = new Scalar(18, 200, 150);
		double runtime = 0;
		double approxDistance;

		List<MatOfPoint> contours = new ArrayList<>();
		MatOfPoint2f approxCurve,contour2f;
		MatOfPoint points;

		//Mat tmp = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat hsvimage = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat grayImage = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat thresh = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat output = new Mat(cameraRes,cameraType, new Scalar(0));
		//Mat dst = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
		Mat blurredImage, morphOutput, croppedImage;
		Mat hierarchy = new Mat();
		Rect rectCrop = new Rect((int) cameraRes.width/2, (int) cameraRes.height/2,
				(int) Math.round(cameraRes.width*0.75),  (int)Math.round(cameraRes.width*0.75));
		//boolean firstRun = false;


		@Override
		public Mat processFrame(Mat input) {
			Log.d("Stage Status","Stage 2 Begin");
			croppedImage = new Mat(input, rectCrop);
			Imgproc.blur(input, blurredImage, new Size(3, 3));
			Imgproc.rectangle(blurredImage,rectCrop,new Scalar(Math.random()));
			Log.d("Stage Status","Stage 2.1 Successful");
			Imgproc.cvtColor(blurredImage, hsvimage, Imgproc.COLOR_RGB2HSV);
			//Imgproc.cvtColor(blurredImage, grayImage, Imgproc.COLOR_RGB2GRAY);
			Log.d("Stage Status","Stage 2.2 Successful");
			Core.inRange(hsvimage,minValues,maxValues, thresh);
			Log.d("Stage Status","Stage 2.3 Successful");

			Imgproc.erode(thresh, thresh, erodeElement);
			Imgproc.erode(thresh, thresh, erodeElement);
			Log.d("Stage Status","Stage 2.4.1 Successful");
			Imgproc.dilate(thresh, thresh, dilateElement);
			Imgproc.dilate(thresh, thresh, dilateElement);
			Log.d("Stage Status","Stage 2.4.2 Successful");

			List<MatOfPoint> contours = new ArrayList<>();
			Imgproc.findContours(thresh, contours, hierarchy,
					Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
			Log.d("Stage Status","Stage 2.5 Successful");

			input.copyTo(output);
			//Imgproc.cvtColor(hsvimage.clone(),output,Imgproc.COLOR_HSV2RGB);
			for (int contourIdx=0; contourIdx < contours.size(); contourIdx++) {
				// Minimum size allowed for consideration
				MatOfPoint2f approxCurve = new MatOfPoint2f();
				MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(contourIdx).toArray());
				Point[] testArray = contours.get(contourIdx).toArray();

				//Processing on mMOP2f1 which is in type MatOfPoint2f
				double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
				Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

				//Convert back to MatOfPoint
				MatOfPoint points = new MatOfPoint(approxCurve.toArray());

				// Get bounding shape of contour
				//Imgproc.HoughCircles();
				Rect rect = Imgproc.boundingRect(points);

				Imgproc.rectangle(output, new Point(rect.x, rect.y),
						new Point(rect.x + rect.width, rect.y + rect.height),
						new Scalar(255, 0, 0, 100), 3);
			}

			Log.d("Stage Status","Stage 2.6 Successful");
			telemetry.addData("Runtime", "%.3f",getRuntime()-runtime);
			telemetry.update();
			runtime = getRuntime();
			Log.d("Stage Status","Stage 2.7 Successful");
			Log.d("Stage Status","Stage 2 Successful");
			return output;
		}

		@Override
		public void onViewportTapped() {
			viewportPaused = !viewportPaused;

			if(viewportPaused) phoneCam.pauseViewport();
			else phoneCam.resumeViewport();
		}
	}
}