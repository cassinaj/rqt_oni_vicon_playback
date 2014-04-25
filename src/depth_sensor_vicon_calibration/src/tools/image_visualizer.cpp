#include "image_visualizer.hpp"

#include <cv.h>
#include <highgui.h>
#include <limits>


namespace vis
{
using namespace Eigen;
using namespace std;


ImageVisualizer::ImageVisualizer(
		const int &n_rows, const int &n_cols)
		:n_rows_(n_rows), n_cols_(n_cols)
{
	image_ = cvCreateImage(cvSize(n_cols_, n_rows_),IPL_DEPTH_8U,3);

	for(int row = 0; row < n_rows_; row++)
		for(int col = 0; col < n_cols_; col++)
		{
			((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 0] = 0;
			((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 1] = 0;
			((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 2] = 0;
		}
}

ImageVisualizer::~ImageVisualizer() {cvReleaseImage(&image_);}

void ImageVisualizer::set_image(
		const std::vector<float> &image,
		const bool &invert_image)
{
	std::vector<float> display_image(image.size());

	// find min and max of image -------------------------------------------------------------------------------------------------
	float max = -numeric_limits<float>::max();
	float min = numeric_limits<float>::max();
	for(size_t i = 0; i < image.size(); i++)
	{
		if(invert_image)
			display_image[i] = 1/image[i];
		else
			display_image[i] = image[i];

		min = display_image[i] < min ? display_image[i] : min;
		max = display_image[i] > max ? display_image[i] : max;
	}
	for(size_t i = 0; i < display_image.size(); i++)
		display_image[i] = (display_image[i] - min) / (max-min) * 255.;


	// fill values from vector into image --------------------------------------------------------------------------------------------
	for(int row = 0; row < n_rows_; row++)
		for(int col = 0; col < n_cols_; col++)
		{
			((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 0] = display_image[row*n_cols_ + col];
			((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 1] = display_image[row*n_cols_ + col];
			((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 2] = display_image[row*n_cols_ + col];
		}
}

void ImageVisualizer::add_points(
		const std::vector<Eigen::Vector3f> &points,
		const Eigen::Matrix3f &camera_matrix,
		const Eigen::Matrix3f &R,
		const Eigen::Vector3f &t,
		const std::vector<float> &colors)
{
	std::vector<int> point_indices(points.size());

	std::vector<float> new_colors;
	if(colors.size() != 0)
		new_colors = colors;
	else
		new_colors.resize(points.size());

	for(size_t i = 0; i < points.size(); i++)
	{
		Vector3f point = R * points[i] + t;
		int row, col;
		Cart2Index(point, camera_matrix, row, col);
		point_indices[i] = row*n_cols_ + col;

		if(colors.size() == 0)
			new_colors[i] = point(2);
	}

	add_points(point_indices, new_colors);
}

void ImageVisualizer::add_points(
		const std::vector<int> &point_indices,
		const std::vector<float> &colors)
{
	// if no color has been given we set it to some value -----------------------------
	vector<float> new_colors;

	if(colors.size() != 0)
		new_colors = colors;
	else
		new_colors = vector<float>(point_indices.size(), 1);

	// renormalize colors -----------------------------
	float max = -numeric_limits<float>::max();
	float min = numeric_limits<float>::max();
	for(int i = 0; i < int(colors.size()); i++)
	{
		min = colors[i] < min ? colors[i] : min;
		max = colors[i] > max ? colors[i] : max;
	}
	if(min == max) min = 0;

	for(int i = 0; i < int(point_indices.size()); i++)
	{
		int row = point_indices[i]/n_cols_;
		int col = point_indices[i]%n_cols_;

		((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 2] = (new_colors[i]-min)/(max-min) * 255.;
		((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 1] = 0;
		((uchar *)(image_->imageData + row*image_->widthStep))[col*3 + 0] = (1 - (new_colors[i]-min)/(max-min)) * 255.;
	}
}

char ImageVisualizer::show_image(
		const std::string &window_name,
		const int &window_width, const int &window_height,
		const int &delay) const
{
	cvNamedWindow(window_name.c_str(), CV_WINDOW_NORMAL);
	cvShowImage(window_name.c_str(), image_);
	cvResizeWindow(window_name.c_str(), window_width, window_height);
	return cvWaitKey(delay);
}

void ImageVisualizer::Cart2Index(
		const Eigen::Vector3f &cart,
		const Eigen::Matrix3f &camera_matrix,
		int &row, int &col) const
{
	Vector3f index_temp = camera_matrix * cart/cart(2);

	row = floor(float(index_temp(1)+0.5f));
	col = floor(float(index_temp(0)+0.5f));
}



}

