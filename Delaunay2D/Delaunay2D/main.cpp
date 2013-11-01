#define high_resolution_timer
#ifdef high_resolution_timer
#include <windows.h>
#include <stdio.h>
double PCFreq = 0.0;
__int64 CounterStart = 0;

void StartCounter()
{
	LARGE_INTEGER li;
	if(!QueryPerformanceFrequency(&li))
		printf("QueryPerformanceFrequency failed!\n");

	PCFreq = double(li.QuadPart)/1000.0;

	QueryPerformanceCounter(&li);
	CounterStart = li.QuadPart;
}
double GetCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart-CounterStart)/PCFreq;
}

#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <iomanip>
#include <string.h>

#include <time.h>

#define OPENGL
#ifdef OPENGL
#include <GL/freeglut.h>
#endif

using namespace cv;
using namespace std;

typedef std::vector<cv::Point> PointVector;

struct Face
{
	Face(int i, int j, int k):x(i), y(j), z(k){}
	int x, y, z;
};

class PointManager
{
public:
	PointManager()
	{
		length = 1;
		pvList.push_back(PointVector());
	}
	std::vector<PointVector> pvList;
	int length;
};

std::vector<cv::Point> g_point_list[4];
std::vector<Face> face_list;

PointManager gManager[3];
//const char* gWindowName[4] = {"hxjN_new", "hxjE_new", "zlpN_new", "zlpE_new"};
const char* gWindowName[4] = {"manN", "manE", "womanN", "womanE"};
cv::Mat gOriImage[3];
cv::Mat gImage[4];

bool gIsAdd = true;
Subdiv2D gSubdiv[3];
Rect gRect;
int gWidth, gHeight;

int gIndex;

namespace warping{
	cv::Point2d point_mapped2d(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3, const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point& q)
	{
		cv::Point2d p;
		double x = ((q.x-q1.x)*(q3.y-q1.y)-(q.y-q1.y)*(q3.x-q1.x))*1.0/((q2.x-q1.x)*(q3.y-q1.y)-(q2.y-q1.y)*(q3.x-q1.x));
		double y = ((q.x-q1.x)*(q2.y-q1.y)-(q.y-q1.y)*(q2.x-q1.x))*1.0/((q3.x-q1.x)*(q2.y-q1.y)-(q3.y-q1.y)*(q2.x-q1.x));
		
		return cv::Point2d(x*p2.x+y*p3.x+(1-x-y)*p1.x, x*p2.y+y*p3.y+(1-x-y)*p1.y);
	}

	cv::Point point_mapped(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3, const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point& q)
	{
		cv::Point2d p = point_mapped2d(p1, p2, p3, q1, q2, q3, q);
		return cv::Point(p.x+0.5, p.y+0.5);
	}

	bool point2d_in_triangle(const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point2d& q)
	{
		double Qx=q.x-q1.x, Qy=q.y-q1.y;
		double Q2x=q2.x-q1.x, Q2y=q2.y-q1.y;
		double Q3x=q3.x-q1.x, Q3y=q3.y-q1.y;
		double x = (Qx*Q3y-Qy*Q3x)*1.0/(Q2x*Q3y-Q2y*Q3x);
		double y = (Qx*Q2y-Qy*Q2x)*1.0/(Q3x*Q2y-Q3y*Q2x);
		if(x > 0 && y > 0 && (x+y) < 1)
			return true;
		else
			return false;
	}

	bool point_in_triangle(const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point& q)
	{
		//return (point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x-0.5, q.y-0.5)) || point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x-0.5, q.y+0.5))
		//	|| point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x+0.5, q.y+0.5)) || point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x+0.5, q.y-0.5)));
		//return (point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x-0.5, q.y-0.5)) || point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x+0.5, q.y+0.5)));
		double Qx=q.x-q1.x, Qy=q.y-q1.y;
		double Q2x=q2.x-q1.x, Q2y=q2.y-q1.y;
		double Q3x=q3.x-q1.x, Q3y=q3.y-q1.y;
		double x = (Qx*Q3y-Qy*Q3x)*1.0/(Q2x*Q3y-Q2y*Q3x);
		double y = (Qx*Q2y-Qy*Q2x)*1.0/(Q3x*Q2y-Q3y*Q2x);
		if(x > 0 && y > 0 && (x+y) < 1)
			return true;
		else
			return false;
	}

	int find_point_in_face(cv::Point& p, int index)
	{
		for(int i = 0; i < face_list.size(); i++)
		{
			cv::Point& p1 = g_point_list[index][face_list[i].x];
			cv::Point& p2 = g_point_list[index][face_list[i].y];
			cv::Point& p3 = g_point_list[index][face_list[i].z];
			//if(PointinTriangle(p1, p2, p3, p, 0.01))
			if(point_in_triangle(p1,p2,p3,p))
				return i;
		}
		return -1;
	}

	cv::Point find_mapped_point(cv::Point& q, int src_image_index, int tgt_image_index, int face_index)
	{
		cv::Point q1 = g_point_list[src_image_index][face_list[face_index].x];
		cv::Point q2 = g_point_list[src_image_index][face_list[face_index].y];
		cv::Point q3 = g_point_list[src_image_index][face_list[face_index].z];
		
		cv::Point p1 = g_point_list[tgt_image_index][face_list[face_index].x];
		cv::Point p2 = g_point_list[tgt_image_index][face_list[face_index].y];
		cv::Point p3 = g_point_list[tgt_image_index][face_list[face_index].z];
		
		return point_mapped(p1, p2, p3, q1, q2, q3, q);
	}

	//void scan_triangle(/*src*/cv::Point& p1, cv::Point& p2, cv::Point& p3, cv::Mat& srcImage, cv::Point& q1, cv::Point& q2, cv::Point& q3, cv::Mat& tgtImage/*tgt*/)
	//{
	//	//y值随着下标的增加而增加
	//	cv::Point p[3], q[3];
	//	int index[3] = {0,1,2};//y increasing
	//	p[0] = p1;
	//	p[1] = p2;
	//	p[2] = p3;
	//	q[0] = q1;
	//	q[1] = q2;
	//	q[2] = q3;
	//	bool xflag = true;//true +; false -
	//	bool yflag = true;//true down; false up
	//	bool gradient = true;//false vertical
	//	/*扫描方法，找到最高点，确定是正扫描还是反扫面，一行一行的扫，直到碰到最后一个点
	//	 *
	//	 */
	//	if(p1.y < p2.y)//sort by y
	//	{
	//		if(p2.y < p3.y)
	//		{
	//			index[0] = 2;
	//			index[1] = 1;
	//			index[2] = 0;
	//			if(p2.x > p1.x)
	//				xflag = false;//1
	//			//2
	//		}
	//		else if(p2.y == p3.y)
	//		{
	//			if(p2.x < p3.x)
	//			{
	//				index[0] = 1;
	//				index[1] = 2;
	//				index[2] = 0;//3 
	//			}
	//			else
	//			{
	//				index[0] = 2;
	//				index[1] = 1;
	//				index[2] = 0; 
	//				xflag = false;//4
	//			}
	//		}
	//		else
	//		{
	//			if(p1.y < p3.y)
	//			{
	//				index[0] = 1;
	//				index[1] = 2;
	//				index[2] = 0; //5
	//				if(p1.x > p3.x)
	//					xflag = false;//6
	//			}
	//			else
	//			{
	//				index[0] = 1;
	//				index[1] = 0;
	//				index[2] = 2;
	//				if(p3.x > p1.x)
	//					xflag = false;//7
	//				//8
	//			}
	//		}
	//	}
	//	else if(p1.y == p2.y)
	//	{
	//		if(p1.y < p3.y)
	//		{
	//			if(p1.x < p2.x)
	//			{
	//				index[0] = 2;
	//				index[1] = 0;
	//				index[2] = 1;//9
	//			}
	//			else
	//			{
	//				index[0] = 2;
	//				index[1] = 1;
	//				index[2] = 0;//10
	//			}
	//		}
	//		else
	//		{
	//			if(p1.x < p2.x)
	//			{
	//				index[0] = 0;
	//				index[1] = 1;
	//				index[2] = 2;//11
	//			}
	//			else
	//			{
	//				index[0] = 1;
	//				index[1] = 0;
	//				index[2] = 2;//12
	//			}
	//		}
	//	}
	//	else
	//	{
	//		if(p2.y > p3.y)
	//		{
	//			index[0] = 1;
	//			index[1] = 0;
	//			index[2] = 2;//12
	//		}
	//	}

	//	if(p[index[0]].y == p[index[1]].y)///set flag
	//	{
	//		yflag = false;
	//		if(p[index[0]].x == p[index[2]].x && p[index[0]].x < p[index[1]].x)
	//		{
	//			xflag = false;
	//		}
	//		else if(p[index[1]].x == p[index[2]].x && p[index[1]].x < p[index[0]].x)
	//		{
	//			xflag = false;
	//		}
	//	}
	//	else
	//	{
	//		if(p[index[1]].y == p[index[2]].y)
	//		{
	//			if(p[index[1]].x < )
	//		}
	//	}
	//}
	//

};


inline bool isInRect(cv::Point& p, Rect& rect)
{
	if(rect.x < p.x && p.x < rect.x+rect.width && rect.y < p.y && p.y < rect.y+rect.height)
		return true;
	return false;
}

static void draw_subdiv_point( Mat& img, Point2f fp, Scalar color )
{
	circle( img, fp, 3, color);
}


static void draw_subdiv( Mat& img, Subdiv2D& subdiv, Scalar delaunay_color )
{
#if 1
	vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	vector<Point> pt(3);


	for( size_t i = 0; i < triangleList.size(); i++ )
	{
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		line(img, pt[0], pt[1], delaunay_color);
		line(img, pt[1], pt[2], delaunay_color);
		line(img, pt[2], pt[0], delaunay_color);
	}
#else
	vector<Vec4f> edgeList;
	subdiv.getEdgeList(edgeList);
	for( size_t i = 0; i < edgeList.size(); i++ )
	{
		Vec4f e = edgeList[i];
		Point pt0 = Point(cvRound(e[0]), cvRound(e[1]));
		Point pt1 = Point(cvRound(e[2]), cvRound(e[3]));
		line(img, pt0, pt1, delaunay_color, 1, LINE_AA, 0);
	}
#endif
}


static void locate_point( Mat& img, Subdiv2D& subdiv, Point2f fp, Scalar active_color )
{
	int e0=0, vertex=0;


	subdiv.locate(fp, e0, vertex);


	if( e0 > 0 )
	{
		int e = e0;
		do
		{
			Point2f org, dst;
			if( subdiv.edgeOrg(e, &org) > 0 && subdiv.edgeDst(e, &dst) > 0 )
				line( img, org, dst, active_color, 3);


			e = subdiv.getEdge(e, Subdiv2D::NEXT_AROUND_LEFT);
		}
		while( e != e0 );
	}

	draw_subdiv_point( img, fp, active_color );
}

static void paint_voronoi( Mat& img, Subdiv2D& subdiv )
{
	vector<vector<Point2f> > facets;
	vector<Point2f> centers;
	subdiv.getVoronoiFacetList(vector<int>(), facets, centers);

	vector<Point> ifacet;
	vector<vector<Point> > ifacets(1);

	for( size_t i = 0; i < facets.size(); i++ )
	{
		ifacet.resize(facets[i].size());
		for( size_t j = 0; j < facets[i].size(); j++ )
			ifacet[j] = facets[i][j];

		Scalar color;
		color[0] = rand() & 255;
		color[1] = rand() & 255;
		color[2] = rand() & 255;
		fillConvexPoly(img, ifacet, color, 8, 0);

		ifacets[0] = ifacet;
		polylines(img, ifacets, true, Scalar());
		circle(img, centers[i], 3, Scalar());
	}
}

void drawPointSet(int index, bool isLine = false)
{
	cv::Mat image;
	Scalar color;
	if(isLine)
		color = CV_RGB(0, 255, 0);
	else
		color = CV_RGB(255, 0, 0);
	gOriImage[index].copyTo(image);
	for(int i = 0; i < gManager[index].pvList.size(); i++)
	{
		int num = gManager[index].pvList[i].size();
		for(int j = 0; j < num; j++)
		{
			printf("%d %d\n", i, j);
			cv::circle(gImage[index], gManager[index].pvList[i][j], 3, color);
			if(isLine)
				cv::line(gImage[index], gManager[index].pvList[i][j], gManager[index].pvList[i][(j+1)%num], CV_RGB(255, 255, 255));
		}
	}
	imshow(gWindowName[index], image);
}

void InsertPointsAndShow(int index)
{
	for(int i = 0; i < gManager[index].pvList.size(); i++)
	{
		int num = gManager[index].pvList[i].size();
		for(int j = 0; j < num; j++)
			gSubdiv->insert(gManager[index].pvList[i][j]);
	}

	vector<Vec6f> triangleList;
	gSubdiv[index].getTriangleList(triangleList);
	vector<Point> pt(3);

	for( size_t i = 0; i < triangleList.size(); i++ )
	{
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		//if(isInRect(pt[0], gRect) && isInRect(pt[1], gRect) && isInRect(pt[2], gRect))
		{
			line(gImage[index], pt[0], pt[1], CV_RGB(255, 255, 0));
			line(gImage[index], pt[1], pt[2], CV_RGB(255, 255, 0));
			line(gImage[index], pt[2], pt[0], CV_RGB(255, 255, 0));
		}
	}
}

bool findPoint(int index, cv::Point& p, PointVector::iterator& iter, int& i)
{
	for(i = 0; i < gManager[index].pvList.size(); i++)
	{
		for(iter = gManager[index].pvList[i].begin(); iter != gManager[index].pvList[i].end(); iter++)
		{
			printf("%d\n", i);
			if(*iter == p)
				return true;
		}
	}
	return false;
}

void processClick(int x, int y, int index)
{
	printf("add point\n");
	cv::Point p(x, y);

	g_point_list[index].push_back(p);

	PointVector::iterator iter;
	int i;
	if(gIsAdd == true)
	{
		if(findPoint(index, p, iter, i) == false)
		{	
			gManager[index].pvList[gManager[index].pvList.size()-1].push_back(p);
			cv::circle(gImage[index], p, 3, CV_RGB(255, 0, 0));
		}
	}
	else//gIsAdd = false
	{
		if(findPoint(index, p, iter, i))
		{
			gManager[index].pvList[i].erase(iter);
			gOriImage[index].copyTo(gImage[index]);
			drawPointSet(index);
		}
	}
}

void mouseEvent1(int evt, int x, int y, int flags, void* param)
{
	if(evt == CV_EVENT_RBUTTONDOWN)
	{
		printf("click (%d,%d)\n", x, y);
		if(gIndex != 0)
			return;
		processClick(x, y, 0);
	}
	else if(evt == CV_EVENT_LBUTTONDOWN)
	{
		printf("click (%d,%d)\n", x, y);
		gIndex = 0;
		printf("change index to %d\n", gIndex);
	}
}

void mouseEvent2(int evt, int x, int y, int flags, void* param)
{
	if(evt == CV_EVENT_RBUTTONDOWN)
	{
		printf("click (%d,%d)\n", x, y);
		if(gIndex != 1)
			return;
		processClick(x, y, 1);
	}
	else if(evt == CV_EVENT_LBUTTONDOWN)
	{
		printf("click (%d,%d)\n", x, y);
		gIndex = 1;
		printf("change index to %d\n", gIndex);
	}
}

void mouseEvent3(int evt, int x, int y, int flags, void* param)
{
	if(evt == CV_EVENT_RBUTTONDOWN)
	{
		printf("click (%d,%d)\n", x, y);
		if(gIndex != 2)
			return;
		processClick(x, y, 2);
	}
	else if(evt == CV_EVENT_LBUTTONDOWN)
	{
		printf("click (%d,%d)\n", x, y);
		gIndex = 2;
		printf("change index to %d\n", gIndex);
	}
}

void help()
{
	printf("s\t------------\tswitch mode\n");
	printf("w\t------------\twrite point set to file(default.poly)\n");
	printf("p\t------------\tshow delaunary triangulation\n");
	//printf("\t------------\t\n");
}

void ERI(cv::Mat& A, cv::Mat& Ap, cv::Mat& B, cv::Mat& Bp)
{
	B.copyTo(Bp);
	for(int i = 0; i < A.rows; i++)
	{
		for(int j = 0; j < A.cols; j++)
		{
			Vec3b a = A.at<Vec3b>(i,j);
			Vec3b ap = Ap.at<Vec3b>(i,j);
			Vec3b b = B.at<Vec3b>(i,j);
			Bp.at<Vec3b>(i,j) = Vec3b(ap[0]*1.0/a[0]*b[0], ap[1]*1.0/a[1]*b[1], ap[2]*1.0/a[2]*b[2]);
			/*Scalar a = Scalar(cvGet2D(&A, i, j));
			Scalar ap = Scalar(cvGet2D(&Ap, i, j));
			Scalar b = Scalar(cvGet2D(&B, i, j));
			cvSet2D(&Bp, i, j, Scalar(ap.val[0]/a.val[0]*b.val[0], ap.val[1]/a.val[1]*b.val[1], ap.val[2]/a.val[2]*b.val[2]));*/
		}
	}
}

void writeFile(const char* filename, int index)
{
	printf("write %s\n", filename);
	int totalNum = 0;
	int count = 0;
	for(int i = 0; i < gManager[index].pvList.size(); i++)
		totalNum += gManager[index].pvList[i].size();
	FILE* fp;
	fopen_s(&fp, filename, "w");
	
	fprintf(fp, "%d 2 0 0\n\n", totalNum);
	count = 0;
	for(int i = 0; i < gManager[index].pvList.size(); i++)
	{
		for(int j = 0; j < gManager[index].pvList[i].size(); j++)
		{
			count++;
			cv::Point& p = gManager[index].pvList[i][j];
			fprintf(fp, "%d %d %d\n", count, p.x, p.y);
		}
		fprintf(fp, "\n");
	}
	
	fprintf(fp, "%d 0\n", totalNum);
	count = 0;
	int base = 1;
	for(int i = 0; i < gManager[index].pvList.size(); i++)
	{
		int num = gManager[index].pvList[i].size();
		for(int j = 0; j < num; j++)
		{
			count++;
			fprintf(fp, "%d %d %d\n", count, base+j, base+(j+1)%num);
		}
		fprintf(fp, "\n");
		base += gManager[index].pvList[i].size();
	}
	
	fprintf(fp, "0\n");
	//fprintf(fp, "1 %d %d\n", gImage[index].rows/2, gImage[index].cols/2);

	fclose(fp);
}

void readFile(const char* filename, int index)
{
	printf("read %s\n", filename);
	char str[255];
	FILE* fp;
	fopen_s(&fp, filename, "r");
	fgets(str, 255, fp);
	sscanf_s(str, "%d", &(gManager[index].length));
	int count = 0;
	while(1)
	{
		printf("preprecount = %d\n", count);
		fgets(str, 255, fp);
		if(str[0] == '\n')
		{
			if(count != 0)
				gManager[index].pvList.push_back(PointVector());
		}
		else
		{
			count++;
			int n, x, y;
			printf("precount = %d\n", count);
			sscanf_s(str, "%d%d%d", &n, &x, &y);
			printf("count = %d\n", count);
			gManager[index].pvList[gManager[index].pvList.size()-1].push_back(cv::Point(x,y));
			if(count == gManager[index].length)
				break;
		}
	}
	fclose(fp);
}

void draw_point_face(int srcindex, int tgtindex)
{
	for(int i = 0; i < g_point_list[srcindex].size(); i++)
		cv::circle(gImage[tgtindex], g_point_list[srcindex][i], 3, CV_RGB(0, 255, 0));
	for(int i = 0; i < face_list.size(); i++)
	{
		Face& f = face_list[i];
		cv::line(gImage[tgtindex], g_point_list[srcindex][f.x], g_point_list[srcindex][f.y], CV_RGB(255, 255, 255));
		cv::line(gImage[tgtindex], g_point_list[srcindex][f.y], g_point_list[srcindex][f.z], CV_RGB(255, 255, 255));
		cv::line(gImage[tgtindex], g_point_list[srcindex][f.z], g_point_list[srcindex][f.x], CV_RGB(255, 255, 255));
	}
}

void draw_point_face(int index = 0)
{
	draw_point_face(index, index);
	/*for(int i = 0; i < g_point_list[index].size(); i++)
		cv::circle(gImage[index], g_point_list[index][i], 3, CV_RGB(0, 255, 0));
	for(int i = 0; i < face_list.size(); i++)
	{
		Face& f = face_list[i];
		cv::line(gImage[index], g_point_list[index][f.x], g_point_list[index][f.y], CV_RGB(255, 255, 255));
		cv::line(gImage[index], g_point_list[index][f.y], g_point_list[index][f.z], CV_RGB(255, 255, 255));
		cv::line(gImage[index], g_point_list[index][f.z], g_point_list[index][f.x], CV_RGB(255, 255, 255));
	}*/
}

void addGroup(int index)
{
	printf("addGroup[%d]\n", gManager[index].pvList.size()+1);
	gManager[index].pvList.push_back(PointVector());
}

int findIndex(float x, float y, int index = 0)
{
	for(int i = 0; i < g_point_list[index].size(); i++)
	{
		cv::Point& p = g_point_list[index][i];
		if(fabs(x-p.x) < 1.0 && fabs(y-p.y) < 1.0)
			return i;
	}
	return -1;
}

int findIndex(cv::Point& p, int index = 0)
{
	for(int i = 0; i < g_point_list[index].size(); i++)
		if(p == g_point_list[index][i])
			return i;
	return -1;
}



void analysis()
{
	printf("analysis\n");

	for(int i = 0; i < g_point_list[0].size(); i++)
	{
		gSubdiv[0].insert(g_point_list[0][i]);
	}

	int length = g_point_list[0].size();
	vector<Vec6f> triangleList;
	gSubdiv[0].getTriangleList(triangleList);
	vector<Point> pt(3);
	int *flags = new int[length];
	memset(flags, 0, length*sizeof(int));
	int count = 0;
	bool *triattr = new bool[triangleList.size()];
	memset(triattr, false, triangleList.size());

	for( size_t i = 0; i < triangleList.size(); i++ )
	{
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		if(isInRect(pt[0], gRect) && isInRect(pt[1], gRect) && isInRect(pt[2], gRect))
		{
			int x = findIndex(t[0], t[1], 0);
			int y = findIndex(t[2], t[3], 0);
			int z = findIndex(t[4], t[5], 0);
			if(x != -1 && y != -1 && z != -1)
			{
				triattr[i] = true;

				flags[x]++;
				flags[y]++;
				flags[z]++;
				//face_list.push_back(Face(x,y,z));
			}
		}
	}
	for(int i = 0; i < length; i++)
		if(flags[i] != 0)
			count++;
	int interval = 0;
	for(int i = 0; i < count; i++)
	{
		for(; flags[i+interval] == 0; interval++)
			;
		g_point_list[0][i] = g_point_list[0][i+interval];
	}
	for(int i = count; i < length; i++)
		g_point_list[0].pop_back();
	for( size_t i = 0; i < triangleList.size(); i++ )
	{
		if(triattr[i] == true)
		{
			Vec6f t = triangleList[i];
			int x = findIndex(t[0], t[1], 0);
			int y = findIndex(t[2], t[3], 0);
			int z = findIndex(t[4], t[5], 0);
			face_list.push_back(Face(x,y,z));
		}
	}

	delete flags;
	delete triattr;
}

void save_file(const char* filename, int index = 0)
{
	printf("save_file\n");
	FILE* fp;
	fopen_s(&fp, filename, "w");
	
	for(int i = 0; i < g_point_list[index].size(); i++)
		fprintf(fp, "v %d %d\n", g_point_list[index][i].x, g_point_list[index][i].y);
	for(int i = 0; i < face_list.size(); i++)
		fprintf(fp, "f %d %d %d\n", face_list[i].x+1, face_list[i].y+1, face_list[i].z+1);

	fclose(fp);
}

void read_file(const char* filename, int index = 0, bool flag = true)
{
	printf("read_file\n");
	char str[256];
	int x, y, z;
	FILE* fp;
	fopen_s(&fp, filename, "r");
	if(face_list.size() > 0)
		flag = false;
	while(fgets(str, 255, fp) != NULL)
	{
		if(str[0] == 'v')
		{
			sscanf_s(str+1, "%d%d", &x, &y);
			g_point_list[index].push_back(cv::Point(x,y));
		}
		else if(str[0] == 'f' && flag)
		{
			sscanf_s(str+1, "%d%d%d", &x, &y, &z);
			face_list.push_back(Face(x-1, y-1, z-1));
		}
	}
	fclose(fp);
}

void move_point()
{
	for(int i = 0; i < g_point_list[0].size(); i++)
	{
		g_point_list[3].push_back(g_point_list[2][i]+g_point_list[1][i]-g_point_list[0][i]);
	}
}

void move_point(int& minx, int& maxx, int& miny, int& maxy)
{
	int x, y;
	for(int i = 0; i < g_point_list[0].size(); i++)
	{
		x = g_point_list[2][i].x+g_point_list[1][i].x-g_point_list[0][i].x;
		y = g_point_list[2][i].y+g_point_list[1][i].y-g_point_list[0][i].y;
		g_point_list[3].push_back(cv::Point(x, y));
		if(x < minx)
			minx = x;
		else if(x > maxx)
			maxx = x;
		if(y < miny)
			miny = y;
		else if(y > maxy)
			maxy = y;
	}
}


#ifdef OPENGL
GLuint texName;
GLubyte *texImage;
GLubyte *newImage;
const int tgtIndex = 3;

void makeTexImage()
{
	uchar* data = gImage[2].data;
	texImage = new GLubyte[gWidth*gHeight*4];
	for(int i = 0; i < gHeight; i++)
	{
		for(int j = 0; j < gWidth; j++)
		{
			//Vec3b& v = gImage[2].at<Vec3b>(i, j);
			texImage[(i*gWidth+j)*4+2] = *(data++);//v.val[2];
			texImage[(i*gWidth+j)*4+1] = *(data++);//v.val[1];
			texImage[(i*gWidth+j)*4] = *(data++);//v.val[0];
			texImage[(i*gWidth+j)*4+3] = 255;
		}
	}
}

void readPixel()
{
	clock_t start = clock();
	clock_t begin = start;
	newImage = new GLubyte[gWidth*gHeight*4];
	//memset(newImage, 0, sizeof(GLubyte)*gWidth*gHeight*4);
	printf("***************\ntime new = %lf\n", (double)(clock()-start)/CLOCKS_PER_SEC);
	start = clock();
	printf("rrrrrrrr\n");
	StartCounter();
	glReadPixels(0, 0, gWidth, gHeight, GL_RGBA, GL_UNSIGNED_BYTE, newImage);
	printf("%lf\n", GetCounter());
	printf("time read = %lf\n", (double)(clock()-start)/CLOCKS_PER_SEC);
	start = clock();
	uchar* data = gImage[3].data;
	for(int i = 0; i < gHeight; i++)
	{
		for(int j = 0; j < gWidth; j++)
		{
			*(data++) = newImage[(i*gWidth+j)*4+2];
			*(data++) = newImage[(i*gWidth+j)*4+1];
			*(data++) = newImage[(i*gWidth+j)*4];
		}
	}
	printf("time convert = %lf\n", (double)(clock()-start)/CLOCKS_PER_SEC);
	printf("time total = %lf\n\n", (double)(clock()-begin)/CLOCKS_PER_SEC);
}

void init(void)
{    
	clock_t start = clock();
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);

	makeTexImage();
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glGenTextures(1, &texName);
	glBindTexture(GL_TEXTURE_2D, texName);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, gWidth, gHeight, 
		0, GL_RGBA, GL_UNSIGNED_BYTE, texImage);
	printf("time1 = %lf\n", (double)(clock()-start)/CLOCKS_PER_SEC);

}
void display()
{
	clock_t start = clock();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, texName);

	double ratiox = 1.0/gWidth, ratioy = 1.0/gHeight;

	glBegin(GL_TRIANGLES);
	for(int i = 0; i < face_list.size(); i++)
	{
		cv::Point& p1 = g_point_list[tgtIndex][face_list[i].x];
		cv::Point& p2 = g_point_list[tgtIndex][face_list[i].y];
		cv::Point& p3 = g_point_list[tgtIndex][face_list[i].z];
		/*glTexCoord2f(p1.x*ratiox, p1.y*ratioy);glVertex2f(p1.x, gHeight-p1.y);
		glTexCoord2f(p2.x*ratiox, p2.y*ratioy);glVertex2f(p2.x, gHeight-p2.y);
		glTexCoord2f(p3.x*ratiox, p3.y*ratioy);glVertex2f(p3.x, gHeight-p3.y);*/
		glTexCoord2f(p1.x*ratiox, p1.y*ratioy);glVertex2f(p1.x, p1.y);
		glTexCoord2f(p2.x*ratiox, p2.y*ratioy);glVertex2f(p2.x, p2.y);
		glTexCoord2f(p3.x*ratiox, p3.y*ratioy);glVertex2f(p3.x, p3.y);
	}
	glEnd();
	//glFlush();
	glDisable(GL_TEXTURE_2D);
	//printf("time2 = %lf\n", (double)(clock()-start)/CLOCKS_PER_SEC);

	readPixel();
	//imshow(gWindowName[3], gImage[3]);
	

	//imshow(gWindowName[0], gImage[0]);
	//imshow(gWindowName[1], gImage[1]);
	//imshow(gWindowName[2], gImage[2]);
	
	//glColor3f(1.0, 0.0, 0.0);
	//glRectf(-0.5, -0.5, 0.5, 0.5);
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	//glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, gWidth, 0, gHeight);
	
	//gluOrtho2D(0, 1, 0, 1);
	//gluPerspective(60.0, (GLfloat) w/(GLfloat) h, 1.0, 30.0);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glTranslatef(0.0, 0.0, -3.6);
}
#endif



int main( int argc, char** argv)
{
	char key;

	gOriImage[0] = cvLoadImageM((std::string(gWindowName[0])+".jpg").c_str());
	gOriImage[1] = cvLoadImageM((std::string(gWindowName[1])+".jpg").c_str());
	gOriImage[2] = cvLoadImageM((std::string(gWindowName[2])+".jpg").c_str());

	gOriImage[0].copyTo(gImage[0]);
	gOriImage[1].copyTo(gImage[1]);
	gOriImage[2].copyTo(gImage[2]);
	

	resize(gImage[1], gImage[1], gImage[0].size());
	resize(gImage[2], gImage[2], gImage[0].size());

	gWidth = gImage[0].size().width;
	gHeight = gImage[0].size().height;

	gRect = Rect(0, 0, gWidth, gHeight);

	gImage[3] = cv::Mat(gHeight, gWidth, CV_8UC3, Scalar(0, 0, 0));
	
	gSubdiv[0] = Subdiv2D(gRect);
	gSubdiv[1] = Subdiv2D(gRect);
	gSubdiv[2] = Subdiv2D(gRect);

	printf("width = %d, height = %d\n", gWidth, gHeight);
	printf("%d\n", gImage[3].step1());

#ifdef OPENGL
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(gWidth, gHeight);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);

	//opengl
	read_file("default0.obj", 0);
	read_file("default1.obj", 1);
	read_file("default2.obj", 2);
	move_point();
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	for(int i = 0; i < 20; i++)
		display();
	printf("qqqq\n");
	//glutKeyboardFunc(keyboard);
	glutMainLoop();
#else

	namedWindow(gWindowName[0]);
	namedWindow(gWindowName[1]);
	namedWindow(gWindowName[2]);
	namedWindow(gWindowName[3]);

	setMouseCallback(gWindowName[0], mouseEvent1);	
	setMouseCallback(gWindowName[1], mouseEvent2);
	setMouseCallback(gWindowName[2], mouseEvent3);

	//ERI(gImage[0], gImage[1], gImage[2], image);
	//cv::circle(gImage[0], cv::Point(159,123), 3, CV_RGB(0,0,255));

	read_file("default0.obj", 0);
	read_file("default1.obj", 1);
	read_file("default2.obj", 2);
	
	int minx = 0, maxx = 0, miny = 0, maxy = 0;
	clock_t begin = clock();
	move_point(minx, maxx, miny, maxy);
	int face_index = -1;
	//bool state = false;//outside
	for(int i = miny; i < maxy; i++)
	{
		for(int j = minx; j < maxx; j++)
		{
			cv::Point p(j, i);
			if(face_index != -1 && /*last point is in triangle[face_index]*/
				warping::point_in_triangle(g_point_list[3][face_list[face_index].x], g_point_list[3][face_list[face_index].y], g_point_list[3][face_list[face_index].z], p))
			{//guess next point is in the same triangle
				cv::Point q = warping::find_mapped_point(p, 3, 2, face_index);
				gImage[3].at<Vec3b>(i, j) = gImage[2].at<Vec3b>(q.y, q.x);
			}
			else//last point is outside the triangle or the current point is not in the last triangle
			{
				if((face_index = warping::find_point_in_face(p, 3)) != -1)
				{
					cv::Point q = warping::find_mapped_point(p, 3, 2, face_index);
					gImage[3].at<Vec3b>(i, j) = gImage[2].at<Vec3b>(q.y, q.x);
				}
			}
		}
	}

	cv:: Mat test = gImage[3].clone();
	imshow("test", test);

	printf("time = %lf\n", (double)(clock()-begin)/CLOCKS_PER_SEC);
	//draw_point_face(3, 2);
	draw_point_face(3);

	while(1)
	{
		imshow(gWindowName[0], gImage[0]);
		imshow(gWindowName[1], gImage[1]);
		imshow(gWindowName[2], gImage[2]);
		imshow(gWindowName[3], gImage[3]);
		if((key = waitKey(15)) > 0)
		{	
			switch (key)
			{
			case 's':
				gIsAdd = !gIsAdd;
				printf("%s  mode\n", gIsAdd ? "ADD" : "DELETE");
				break;
			case 'p':
				InsertPointsAndShow(gIndex);
				break;
			case 'q':
				//analysis();
				break;
			case 'w':
				save_file((std::string("default")+".obj").c_str(), gIndex);
				//writeFile((std::string("default")+str+".poly").c_str(), gIndex);
				break;
			case 'a':
				analysis();
				//addGroup(gIndex);
				break;
			case 'd':
				drawPointSet(gIndex, true);
				break;
			case 'r':
				read_file((std::string("default")+".obj").c_str(), gIndex);
				draw_point_face(gIndex);
				//readFile((std::string("default")+".poly").c_str(), gIndex);
				//drawPointSet(gIndex);
				break;
			case 'R':
				read_file((std::string("default")+".obj").c_str(), gIndex, false);
				draw_point_face(gIndex);
				break;
			case 27:
				exit(0);
			default:
				break;
			}
		}
	}
#endif

	return 0;
}

