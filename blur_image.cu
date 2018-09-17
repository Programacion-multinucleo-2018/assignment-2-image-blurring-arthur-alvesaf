// Arthur Alves Araujo Ferreira
// A01022593
// Compile with nvcc -o blur blur_image.cu -lopencv_core -lopencv_highgui -lopencv_imgproc

// Includes
#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// input - input image one dimensional array
// ouput - output image one dimensional array
// width, height - width and height of the images
// colorWidthStep - number of color bytes (cols * colors)
// grayWidthStep - number of gray bytes
__global__ void blur_kernel(unsigned char* input, unsigned char* output, int width, int height, int colorWidthStep, int grayWidthStep)
{
	// 2D Index of current thread
	const int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
	const int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

	// Only valid threads perform memory I/O
	if ((xIndex < width) && (yIndex < height))
	{
		//Location of colored pixel in input
		const int color_tid = yIndex * colorWidthStep + (3 * xIndex);

		// Variable initialization
		char resultantBlue = 0;
		char resultantGreen = 0;
		char resultantRed = 0;
		int shiftIdx = 0;
		// Sum average colors around
		// Iterate horizontally and vertically around color_tid
		for (int xOff = -2; xOff < 3; xOff++) {
			for (int yOff = -2; yOff < 3; yOff++) {
				shiftIdx = color_tid+(xOff*3)+(yOff*width*3);
				resultantBlue += input[shiftIdx]*1/26.f;
				resultantGreen += input[shiftIdx+1]*1/26.f;
				resultantRed += input[shiftIdx+2]*1/26.f;
			}
		}

		// Save resulting pixel to output
		output[color_tid]   = static_cast<unsigned char>(resultantBlue);
		output[color_tid+1] = static_cast<unsigned char>(resultantGreen);
		output[color_tid+2] = static_cast<unsigned char>(resultantRed);
	}
}

// Box blur function given an opencv mat and output
void blur_image(const cv::Mat& input, cv::Mat& output)
{
	cout << "Input image step: " << input.step << " rows: " << input.rows << " cols: " << input.cols << endl;

	// Calculate total number of bytes of input and output image
	// Step = cols * number of colors
	size_t bytes = input.step * input.rows;

	unsigned char *d_input, *d_output;

	// Allocate device memory
	cudaMalloc<unsigned char>(&d_input, bytes);
	cudaMalloc<unsigned char>(&d_output, bytes);

	// Copy data from OpenCV input image to device memory
	cudaMemcpy(d_input, input.ptr(), bytes, cudaMemcpyHostToDevice);

	// Specify a reasonable block size
	const dim3 block(16, 16);

	// Calculate grid size to cover the whole image
	// const dim3 grid((input.cols + block.x - 1) / block.x, (input.rows + block.y - 1) / block.y);
	const dim3 grid((int)ceil((float)input.cols / block.x), (int)ceil((float)input.rows/ block.y));
	// printf("blur_kernel<<<(%d, %d) , (%d, %d)>>>\n", grid.x, grid.y, block.x, block.y);

	double total = 0;
	//Call the wrapper function
	for (int i = 0; i < 1; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		// blur_image(input, output);
		blur_kernel <<<grid, block >>>(d_input, d_output, input.cols, input.rows, static_cast<int>(input.step), static_cast<int>(output.step));
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> duration_ms = end - start;
		total += duration_ms.count();
	}

	cout << "GPU Blur in " << total/1 << " ms." << endl;

	// Synchronize to check for any kernel launch errors
	cudaDeviceSynchronize();

	// Copy back data from destination device meory to OpenCV output image
	cudaMemcpy(output.ptr(), d_output, bytes, cudaMemcpyDeviceToHost);

	// Free the device memory
	cudaFree(d_input);
	cudaFree(d_output);
}

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

	//Call the wrapper function
	blur_image(input, output);

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
