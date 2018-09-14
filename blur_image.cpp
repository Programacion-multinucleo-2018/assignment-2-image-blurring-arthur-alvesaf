// Arthur Alves Araujo Ferreira
// A01022593
// Compile with g++ -o blur blur_image.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc

// Includes
#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// Box blur function given an opencv mat and output
void blur_image(const cv::Mat& input, cv::Mat& output)
{
	cout << "Input image step: " << input.step << " rows: " << input.rows << " cols: " << input.cols << endl;

	// Iterate on x
	for (int xIndex=0; xIndex<input.cols; xIndex++) {
	// Iterate on y
		for (int yIndex=0; yIndex<input.rows; yIndex++) {
			// Index of pixel being changed in list
			const int color_tid = yIndex * input.step + (3 * xIndex);

			// Variable initialization
			char resultantBlue = 0;
			char resultantGreen = 0;
			char resultantRed = 0;
			int shiftIdx = 0;


			// Sum average colors around
			// Iterate horizontally and vertically around color_tid
			for (int xOff = -2; xOff < 3; xOff++) {
				for (int yOff = -2; yOff < 3; yOff++) {
					shiftIdx = color_tid+(xOff*3)+(yOff*input.cols*3);

					resultantBlue += input.data[shiftIdx]*1/26.f;
					resultantGreen += input.data[shiftIdx+1]*1/26.f;
					resultantRed += input.data[shiftIdx+2]*1/26.f;
				}
			}

			// cout <<"R "<<resultantRed<<", G "<<resultantGreen<<", B "<<resultantBlue<<endl;
			// Save resulting pixel to output
			output.data[color_tid]   = static_cast<unsigned char>(resultantBlue);
			output.data[color_tid+1] = static_cast<unsigned char>(resultantGreen);
			output.data[color_tid+2] = static_cast<unsigned char>(resultantRed);
		}
	}
}

// Main function
int main(int argc, char *argv[])
{
	// Variable initialization
	string imagePath;

	// Check for program inputs
	if(argc < 2)
		imagePath = "image.jpg";
  	else
  		imagePath = argv[1];

	// Read input image from the disk
	cv::Mat input = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);

	if (input.empty())
	{
		cout << "Image Not Found!" << std::endl;
		cin.get();
		return -1;
	}

	//Create output image
	cv::Mat output(input.rows, input.cols, input.type());

	double total = 0;
	//Call the wrapper function
	for (int i = 0; i < 20; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		blur_image(input, output);
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> duration_ms = end - start;
		total += duration_ms.count();
	}
	cout << "CPU Blur in " << total/20 << " ms." << endl;

	//Allow the windows to resize
	namedWindow("Input", cv::WINDOW_NORMAL);
	namedWindow("Output", cv::WINDOW_NORMAL);

	//Show the input and output
	imshow("Input", input);
	imshow("Output", output);

	//Wait for key press
	cv::waitKey();

	return 0;
}
