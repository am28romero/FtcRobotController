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

package org.firstinspires.ftc.teamcode.imageproc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name="Object Tracking", group="Snippets")
public class ObjectTracking extends LinearOpMode {
	OpenCvCamera phoneCam;
	final Size cameraRes = new Size(320*2, 240*2);
	final int cameraType = 0;

	@Override
	public void runOpMode() {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				phoneCam.startStreaming((int) cameraRes.width, (int) cameraRes.height, OpenCvCameraRotation.UPRIGHT);
			}
		});

		waitForStart();
		resetStartTime();
		phoneCam.setPipeline(new Pipeline());
		while (opModeIsActive()) {
			if(gamepad1.start && gamepad1.back) phoneCam.stopStreaming();
			sleep(100);
		}
	}

	class Pipeline extends OpenCvPipeline {
		boolean viewportPaused = false;
		final Scalar maxValues = new Scalar(50, 255, 255);
		final Scalar minValues = new Scalar(15, 150, 150);
		double approxDistance;

		List<MatOfPoint> contours = new ArrayList<>();
		MatOfPoint2f approxCurve,contour2f;
		Mat tmp = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat output = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat hierarchy = new Mat();
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
		Rect boundingRect;


		@Override
		public Mat processFrame(Mat input) {
			Imgproc.blur(input, tmp, new Size(3, 3));
			Imgproc.cvtColor(tmp, tmp, Imgproc.COLOR_RGB2HSV);
			Core.inRange(tmp,minValues,maxValues, tmp);

			Imgproc.erode(tmp, tmp, erodeElement);
			Imgproc.erode(tmp, tmp, erodeElement);
			Imgproc.dilate(tmp, tmp, dilateElement);
			Imgproc.dilate(tmp, tmp, dilateElement);

			contours = new ArrayList<>();
			Imgproc.findContours(tmp, contours, hierarchy,
					Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			output.copySize(input);
			for (int contourIdx=0; contourIdx < contours.size(); contourIdx++) {
				approxCurve = new MatOfPoint2f();
				contour2f = new MatOfPoint2f(contours.get(contourIdx).toArray());

				approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
				Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);
				boundingRect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));

				Imgproc.rectangle(output, new Point(boundingRect.x, boundingRect.y),
						new Point(boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height),
						new Scalar(255, 0, 0, 100), 3);
			}

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