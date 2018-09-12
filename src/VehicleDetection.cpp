/*
 * VehicleDetection.cpp
 *
 *  Created on: 28-Aug-2014
 *      Author: Guru Prasad
 */
#include "vechile.h"

void mouseEvent(int, int, int, int, void*);
void mouseEventLine(int, int, int, int, void*);
void drawBox(Point, Point, Mat&);
void drawLine(Point, Point, Mat&);

Mat frame, back, fore;
Mat original, draw_lineImg;
Mat cropImg;
int count = 0;
float ratio_row, ratio_col;
vector<Rect> boundRect;

bool isDrawing = false, first_fr = true;
Point start, end, l_start[nos_line], l_end[nos_line];
int line_y;

int main(int argc, char *argv[]) {

	int count = 0;
	char text[18];
	int locality = 0;
	float ratio_row, ratio_col;
	bool first_time = true;

	//time variable
	double prev_time[nos_line] = { };
	double new_time[nos_line] = { getTickCount(), getTickCount(),
			getTickCount(), getTickCount() };
	double common_prev_time = 0;
	double time_difference = 0;
	double time_thres = 400;
	float avg_difference = 0;

	VideoCapture cap(argv[1]);
	cap >> frame;
	ratio_row = (float) frame.rows / 320 * 1.1;
	ratio_col = (float) frame.cols / 480;
	resize(frame, frame, cvSize(480, 320));
	frame.copyTo(draw_lineImg);

	initscr();
	refresh();
	namedWindow("ROI", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("ROI", mouseEvent, &frame);
	imshow("ROI", frame);

	waitKey(0);
	destroyWindow("ROI");

	draw_lineImg = frame(Rect(start, end));
	namedWindow("Line", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("Line", mouseEventLine, &draw_lineImg);
	imshow("Line", draw_lineImg);

	waitKey(0);
	destroyWindow("Line");

	medianBlur(frame, frame, 7);

	BackgroundSubtractorMOG2 bg(100, 150, false); // history is an int, distance_threshold is an int (usually set to 16), shadow_detection is a bool
	bg(frame, fore, -1); //learning_rate = -1 here
	Mat const structure_elem = getStructuringElement(MORPH_RECT, Size(3, 3));

	for (;;) {
		cap >> frame;
		if (!frame.data) {
			endwin();
			std::printf("End of frame. exiting....\n");

			exit(EXIT_SUCCESS);
		}
		frame.copyTo(original);
		resize(frame, frame, cvSize(480, 320));

		Mat roi = frame(Rect(start, end));
		medianBlur(roi, roi, 3);
		bg.operator ()(roi, fore);
		//morphologyEx(fore,fore, MORPH_OPEN,structure_elem);
		//morphologyEx(fore,fore, MORPH_CLOSE,structure_elem);

		//draw created rectangle and line
		line(frame, Point(start.x, line_y + start.y),
				Point(end.x, line_y + start.y), Scalar(100, 255, 50), 1);
		rectangle(frame, start, end, Scalar(255, 0, 0), 1, 8, 0);

		//find contour in the image
		unsigned int contour_length_threshold = 150;
		std::vector<std::vector<cv::Point> > contours;
		findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//remove smaller  contours
		for (vector<vector<Point> >::iterator it = contours.begin();
				it != contours.end();) {
			if (it->size() < contour_length_threshold)
				it = contours.erase(it);
			else
				++it;
		}

		//create boundingRect around the contours.
		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++) {
			approxPolyDP(Mat(contours[i]), contours_poly[i], 5, true);
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
		}

		//create rectangle over detected contours.
		for (unsigned int i = 0; i < contours.size(); i++) {
			Point center = (boundRect[i].br() + boundRect[i].tl());
			center *= 0.5;
			rectangle(frame, boundRect[i].tl() + start,
					boundRect[i].br() + start, Scalar(255, 255, 255), 2, 2, 0);
			int radius = 20;
			circle(frame, center + start, 2, Scalar(0, 0, 255), -1, CV_AA);

			if (first_fr) {
				first_fr = false;
				continue;
			}

			//check position of center of contour with respect to line position.
			if ((line_y - center.y) * (line_y - center.y) < radius * radius) {
				locality = 0;
				if (center.x < 1 * (roi.cols / nos_line))
					locality = 1;
				else if (center.x < 2 * (roi.cols / nos_line))
					locality = 2;
				else if (center.x < 3 * (roi.cols / nos_line))
					locality = 3;
				else if (center.x < 4 * (roi.cols / nos_line))
					locality = 4;

				switch (locality) {
				case 1:
					new_time[0] = (double) getTickCount(); //current tick count
					if ((((new_time[0] - prev_time[0]) / getTickFrequency())
							* 1000) > time_thres) {

						count++;
						line(frame, Point(start.x, line_y + start.y),
								Point(end.x, line_y + start.y),
								Scalar(100, 10, 100), 2);
						time_difference = (new_time[0] - common_prev_time)
								/ getTickFrequency();
						common_prev_time = new_time[0];
						avg_difference = time_difference / count;
						move(10, 20);
						if (first_time) {
							printw("Total cars = %d   interval = N/A", count);
							first_time = FALSE;
						} else
							printw("Total cars = %d   interval = %2.2f", count,
									time_difference);
						refresh();
						prev_time[0] = new_time[0];

						//find cordinate of ractange on original image from ROI image
						cropImg =
								original(
										Rect(
												Point(
														(int) (boundRect[i].tl()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].tl()
																+ start).y
																* ratio_row),
												Point(
														(int) (boundRect[i].br()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].br()
																+ start).y
																* ratio_row)));
						std::string s;
						std::stringstream out;
						out << "./images/Car-" << count << ".jpg";
						s = out.str();
						imwrite(s, cropImg);

					}
					break;
				case 2:
					new_time[1] = (double) getTickCount(); //current tick count
					if ((((new_time[1] - prev_time[1]) / getTickFrequency())
							* 1000) > time_thres) {

						count++;
						line(frame, Point(start.x, line_y + start.y),
								Point(end.x, line_y + start.y),
								Scalar(100, 10, 100), 2);
						time_difference = (new_time[1] - common_prev_time)
								/ getTickFrequency();
						common_prev_time = new_time[1];
						avg_difference = time_difference / count;
						move(10, 20);
						if (first_time) {
							printw("Total cars = %d   interval = N/A", count);
							first_time = FALSE;
						} else
							printw("Total cars = %d   interval = %2.2f", count,
									time_difference);
						refresh();
						prev_time[1] = new_time[1];

						//find cordinate of ractange on original image from ROI image
						cropImg =
								original(
										Rect(
												Point(
														(int) (boundRect[i].tl()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].tl()
																+ start).y
																* ratio_row),
												Point(
														(int) (boundRect[i].br()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].br()
																+ start).y
																* ratio_row)));
						std::string s;
						std::stringstream out;
						out << "./images/Car-" << count << ".jpg";
						s = out.str();
						imwrite(s, cropImg);

					}
					break;
				case 3:
					new_time[2] = (double) getTickCount(); //current tick count
					if ((((new_time[2] - prev_time[2]) / getTickFrequency())
							* 1000) > time_thres) {
						count++;
						line(frame, Point(start.x, line_y + start.y),
								Point(end.x, line_y + start.y),
								Scalar(100, 10, 100), 2);
						time_difference = (new_time[2] - common_prev_time)
								/ getTickFrequency();
						common_prev_time = new_time[2];
						avg_difference = time_difference / count;
						move(10, 20);
						if (first_time) {
							printw("Total cars = %d   interval = N/A", count);
							first_time = FALSE;
						} else
							printw("Total cars = %d   interval = %2.2f", count,
									time_difference);
						refresh();
						prev_time[2] = new_time[2];

						//find cordinate of ractange on original image from ROI image
						cropImg =
								original(
										Rect(
												Point(
														(int) (boundRect[i].tl()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].tl()
																+ start).y
																* ratio_row),
												Point(
														(int) (boundRect[i].br()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].br()
																+ start).y
																* ratio_row)));
						std::string s;
						std::stringstream out;
						out << "./images/Car-" << count << ".jpg";
						s = out.str();
						imwrite(s, cropImg);

					}
					break;
				case 4:
					new_time[2] = (double) getTickCount(); //current tick count
					if ((((new_time[3] - prev_time[3]) / getTickFrequency())
							* 1000) > time_thres) {
						count++;
						line(frame, Point(start.x, line_y + start.y),
								Point(end.x, line_y + start.y),
								Scalar(100, 10, 100), 2);
						time_difference = (new_time[3] - common_prev_time)
								/ getTickFrequency();
						common_prev_time = new_time[3];
						avg_difference = time_difference / count;
						move(10, 20);
						if (first_time) {
							printw("Total cars = %d   interval = N/A", count);
							first_time = FALSE;
						} else
							printw("Total cars = %d   interval = %2.2f", count,
									time_difference);
						refresh();
						prev_time[3] = new_time[3];

						//find cordinate of ractange on original image from ROI image
						cropImg =
								original(
										Rect(
												Point(
														(int) (boundRect[i].tl()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].tl()
																+ start).y
																* ratio_row),
												Point(
														(int) (boundRect[i].br()
																+ start).x
																* ratio_col,
														(int) (boundRect[i].br()
																+ start).y
																* ratio_row)));
						std::string s;
						std::stringstream out;
						out << "./images/Car-" << count << ".jpg";
						s = out.str();
						imwrite(s, cropImg);
					}
					break;

				} //end of switch-case
			}

		} //end of for loop

		  //print on the window
		snprintf(text, 18, "No of cars = %d", count);
		putText(frame, text, Point(frame.cols - 155, frame.rows - 5),
				CV_FONT_HERSHEY_PLAIN, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

		//display frame
		imshow("Frame", frame);
		if (cv::waitKey(60) >= 0)
			break;
	}
	endwin();
	printf("\n\n\n\n");
	return 0;
}

//Draw line on the video using mouse selection
void drawLine(Point start, Point end, Mat& img) {
	Mat temp(img.size(), img.type());
	img.copyTo(temp);
	Scalar color = Scalar(0, 255, 0);
	line(temp, Point(0, line_y), Point(temp.cols, line_y), color, 1, 8, 0);
	imshow("Line", temp);
	return;

}

//Anevent method used to decttect drawing line
void mouseEventLine(int evt, int x, int y, int flag, void* param) {

	if (evt == CV_EVENT_LBUTTONDOWN) {
		isDrawing = true;
		return;
	}
	if (evt == CV_EVENT_LBUTTONUP) {
		isDrawing = false;
		cv::Mat* image = static_cast<cv::Mat *>(param);

		line_y = y;
		drawLine(start, end, *image);
		return;
	}

	if (isDrawing && evt == CV_EVENT_MOUSEMOVE) {
		line_y = y;
		cv::Mat* image = static_cast<cv::Mat *>(param);
		drawLine(start, end, *image);
	}
}

//Draw box on the videe to create ROI
void drawBox(Point start, Point end, Mat& img) {
	Mat temp(img.size(), img.type());
	img.copyTo(temp);
	Scalar color = Scalar(0, 255, 0);
	rectangle(temp, start, end, color, 1, 8, 0);
	imshow("ROI", temp);
	return;
}

//An event method to detect drawing box
void mouseEvent(int evt, int x, int y, int flag, void* param) {
	if (evt == CV_EVENT_LBUTTONDOWN) {
		isDrawing = true;
		start.x = x;
		start.y = y;
		return;
	}
	if (evt == CV_EVENT_LBUTTONUP) {
		isDrawing = false;
		end.x = x;
		end.y = y;
		cv::Mat* image = static_cast<cv::Mat *>(param);
		drawBox(start, end, *image);
		return;
	}
	if (isDrawing && evt == CV_EVENT_MOUSEMOVE) {
		end.x = x;
		end.y = y;
		cv::Mat* image = static_cast<cv::Mat *>(param);
		drawBox(start, end, *image);
	}
}
