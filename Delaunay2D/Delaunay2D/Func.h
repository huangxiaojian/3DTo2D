#ifndef FUNC_H
#define FUNC_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <iomanip>


struct Face
{
	Face(int i, int j, int k):x(i),y(j),z(k){}
	Face():x(0),y(0),z(0){}
	int x, y, z;
};

class Triangulation
{
public:
	Triangulation(cv::Mat* img)
	{
		image = img;
		width = image->cols;
		height = image->rows; 

		net_radius = 30;
		eps = 0.0001;
	}

	void read_file(const char* filename);
	void set_connect();
	bool is_connect(int p1, int p2);

	void prerun();
	void run();
	
	double cal_error(std::map<int, int> &cor_x,std::map<int, int> &cor_y);
	void multi_grid_solver_newton();
	void insert_points();


	std::vector<cv::Point> point_list;
	std::vector<Face> face_list;
	std::vector<bool> attri_list;
	//true hard, false soft

	cv::Mat* image;
	int width;
	int height;

	cv::Mat* mat_connect;

	int iter_flag;//
	int iter_count;//
	int net_radius;//
	double error;
	double eps;//

	std::map<int, int> x_soft_IndexMat;	
	std::map<int, int> y_soft_IndexMat;

	std::map<int, int> x_soft_MatIndex;
	std::map<int, int> y_soft_MatIndex;


	std::map<int, int> x_hard_IndexMat;
	std::map<int, int> y_hard_IndexMat;

	std::map<int, int> x_hard_MatIndex;
	std::map<int, int> y_hard_MatIndex;

	std::map<int, int> x_back_Position;
	std::map<int, int> y_back_Position;

	/*std::map<int, int> con_PointIndex;
	std::map<int, int> con_IndexPoint;
	
	std::map<int, double> RowRatio;
	std::map<int, double> ColRatio;
	*/
	std::map<int, int> x_Position;
	std::map<int, int> y_Position;
};

#endif