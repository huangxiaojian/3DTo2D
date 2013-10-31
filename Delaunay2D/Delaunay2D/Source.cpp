#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <set>

using namespace cv;
using namespace std;

std::set<int> gPSet[3];
const char* gWindowName[3] = {"hxjN", "hxjE", "zlpN"};
cv::Mat gOriImage[3];
cv::Mat gImage[3];

bool gIsAdd = true;
Subdiv2D gSubdiv[3];
Rect gRect;
int gWidth;

inline bool isInRect(cv::Point& p, Rect& rect)
{
	if(rect.x < p.x && p.x < rect.x+rect.width && rect.y < p.y && p.y < rect.y+rect.height)
		return true;
	return false;
}

inline int position(int x, int y)
{
	return (y*gWidth+x);
}

inline cv::Point getPoint(int pos)
{
	return cv::Point(pos%gWidth, pos/gWidth);
}

void PrintSet(int index)
{
	std::set<int>::iterator iter;
	for(iter = gPSet[index].begin(); iter != gPSet[index].end(); iter++)
		printf("<%d,%d>\t", getPoint(*iter).x, getPoint(*iter).y);
	printf("\n");
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

void drawPointSet(int index)
{
	std::set<int>::iterator iter;
	for(iter = gPSet[index].begin(); iter != gPSet[index].end(); iter++)
		cv::circle(gImage[index], getPoint(*iter), 3, Scalar());
}

void InsertPointsAndShow(int index)
{
	std::set<int>::iterator iter;
	for(iter = gPSet[index].begin(); iter != gPSet[index].end(); iter++)
		gSubdiv[index].insert(getPoint(*iter));

	vector<Vec6f> triangleList;
	gSubdiv[index].getTriangleList(triangleList);
	vector<Point> pt(3);


	for( size_t i = 0; i < triangleList.size(); i++ )
	{
		Vec6f t = triangleList[i];
		if(isInRect(cv::Point(t[0], t[1]), gRect) && isInRect(cv::Point(t[2], t[3]), gRect) && isInRect(cv::Point(t[4], t[5]), gRect))
		{
			pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
			pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
			pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
			line(gImage[index], pt[0], pt[1], CV_RGB(255, 255, 0));
			line(gImage[index], pt[1], pt[2], CV_RGB(255, 255, 0));
			line(gImage[index], pt[2], pt[0], CV_RGB(255, 255, 0));
		}
	}
}

void processKey(int x, int y, int index)
{
	cv::Point p(x, y);
	if(gIsAdd == true)
	{
		gPSet[index].insert(position(x,y));
		//gSubdiv[index].insert(cv::Point(x,y));
		cv::circle(gImage[index], cv::Point(x,y), 3, CV_RGB(255, 0, 0));
	}
	else//gIsAdd = false
	{
		gPSet[index].erase(position(x,y));
		gOriImage[index].copyTo(gImage[index]);
		drawPointSet(index);
	}
}

void mouseEvent1(int evt, int x, int y, int flags, void* param)
{
    if(evt == CV_EVENT_LBUTTONDOWN)
	{
        processKey(x, y, 0);
		PrintSet(0);
    }
}

void mouseEvent2(int evt, int x, int y, int flags, void* param)
{
    if(evt == CV_EVENT_LBUTTONDOWN)
	{
        processKey(x, y, 1);
		PrintSet(1);
    }
}

void mouseEvent3(int evt, int x, int y, int flags, void* param)
{
    if(evt == CV_EVENT_LBUTTONDOWN)
	{
        processKey(x, y, 2);
		PrintSet(2);
    }
}

void help()
{
	printf("s\t------------\tswitch mode\n");
	printf("w\t------------\twrite point set to file(default.poly)\n");
	printf("p\t------------\tshow delaunary triangulation\n");
	//printf("\t------------\t\n");
}

void writeFile(const char* filename, int index)
{
	int num = gPSet[index].size();
	FILE* fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "%d 2 1 0\n", num);
	//for()
	fclose(fp);
}

int main( int, char** )
{
	std::set<int> tset;
	tset.insert(10);
	tset.insert(5);
	tset.insert(8);
	tset.insert(78);
	tset.insert(9);
	tset.insert(1);
	for(std::set<int>::iterator iter = tset.begin(); iter != tset.end(); iter++)
		printf("%d\n", *iter);

	char key;

	gOriImage[0] = cvLoadImageM((std::string(gWindowName[0])+".jpg").c_str());
	gOriImage[1] = cvLoadImageM((std::string(gWindowName[1])+".jpg").c_str());
	gOriImage[2] = cvLoadImageM((std::string(gWindowName[2])+".jpg").c_str());

	gOriImage[0].copyTo(gImage[0]);
	gOriImage[1].copyTo(gImage[1]);
	gOriImage[2].copyTo(gImage[2]);

	resize(gImage[1], gImage[1], gImage[0].size());
	resize(gImage[2], gImage[2], gImage[0].size());

	gRect = Rect(0, 0, gImage[0].size().width, gImage[0].size().height);

	gSubdiv[0] = Subdiv2D(gRect);
	gSubdiv[1] = Subdiv2D(gRect);
	gSubdiv[2] = Subdiv2D(gRect);

	gWidth = gImage[0].size().width;

	namedWindow(gWindowName[0]);
	//namedWindow(gWindowName[1]);
	//namedWindow(gWindowName[2]);

	setMouseCallback(gWindowName[0], mouseEvent1);	
	//setMouseCallback(gWindowName[1], mouseEvent2);
	//setMouseCallback(gWindowName[2], mouseEvent3);

	while(1)
	{
		imshow(gWindowName[0], gImage[0]);
		//imshow(gWindowName[1], gImage[1]);
		//imshow(gWindowName[2], gImage[2]);
		if((key = waitKey(15)) > 0)
		{	
			if(key == 's')
			{
				gIsAdd = !gIsAdd;
				printf("%s  mode\n", gIsAdd ? "ADD" : "DELETE");
			}
			if(key == 'p')
			{
				InsertPointsAndShow(0);
			}
			if(key == 'w')
			{
				writeFile("default1.poly", 0);
			}
			if(key == 'a')
			{
				//writeFile("default2.poly", 1);
			}
			if(key == 27)
				break;
		}
	}

	

	return 0;
}

