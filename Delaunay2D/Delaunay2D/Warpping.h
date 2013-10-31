#ifndef WARPING_H
#define WARPING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <iomanip>

struct Face
{
	Face(int i, int j, int k):x(i), y(j), z(k){}
	int x, y, z;
};

class Warpping{
	cv::Point2d PointMapped2d(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3, const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point& q)
	{
		cv::Point2d p;
		double x = ((q.x-q1.x)*(q3.y-q1.y)-(q.y-q1.y)*(q3.x-q1.x))*1.0/((q2.x-q1.x)*(q3.y-q1.y)-(q2.y-q1.y)*(q3.x-q1.x));
		double y = ((q.x-q1.x)*(q2.y-q1.y)-(q.y-q1.y)*(q2.x-q1.x))*1.0/((q3.x-q1.x)*(q2.y-q1.y)-(q3.y-q1.y)*(q2.x-q1.x));

		return cv::Point2d(x*p2.x+y*p3.x+(1-x-y)*p1.x, x*p2.y+y*p3.y+(1-x-y)*p1.y);
	}

	cv::Point PointMapped(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3, const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point& q)
	{
		cv::Point2d p = PointMapped2d(p1, p2, p3, q1, q2, q3, q);
		return cv::Point(p.x+0.5, p.y+0.5);
	}

	bool Point2dInTriangle(const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point2d& q)
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

	bool PointInTriangle(const cv::Point& q1, const cv::Point& q2, const cv::Point& q3, const cv::Point& q)
	{
		//return (point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x-0.5, q.y-0.5)) || point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x-0.5, q.y+0.5))
		//	|| point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x+0.5, q.y+0.5)) || point2d_in_triangle(q1,q2,q3,cv::Point2d(q.x+0.5, q.y-0.5)));
		return (Point2dInTriangle(q1,q2,q3,cv::Point2d(q.x-0.5, q.y-0.5)) || Point2dInTriangle(q1,q2,q3,cv::Point2d(q.x+0.5, q.y+0.5)));
	}

	int FindPointInFace(cv::Point& p, std::vector<cv::Point>& pointList)
	{
		for(int i = 0; i < m_faces.size(); i++)
		{
			cv::Point& p1 = pointList[m_faces[i].x];
			cv::Point& p2 = pointList[m_faces[i].y];
			cv::Point& p3 = pointList[m_faces[i].z];
			if(PointInTriangle(p1,p2,p3,p))
				return i;
		}
		return -1;
	}

	cv::Point FindMappedPoint(cv::Point& q, std::vector<cv::Point>& srcPointList, std::vector<cv::Point>& tgtPointList, int face_index)
	{
		cv::Point q1 = srcPointList[m_faces[face_index].x];
		cv::Point q2 = srcPointList[m_faces[face_index].y];
		cv::Point q3 = srcPointList[m_faces[face_index].z];

		cv::Point p1 = tgtPointList[m_faces[face_index].x];
		cv::Point p2 = tgtPointList[m_faces[face_index].y];
		cv::Point p3 = tgtPointList[m_faces[face_index].z];

		return PointMapped(p1, p2, p3, q1, q2, q3, q);
	}
	void CalBoundary()
	{
		int x, y;
		for(int i = 0; i < m_tgtPoints.size(); i++)
		{
			x = m_tgtPoints[i].x;
			y = m_tgtPoints[i].y;
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
public:
	void Warp()
	{
		CalBoundary();
		int face_index = -1;
		//bool state = false;//outside
		for(int i = miny; i < maxy; i++)
		{
			for(int j = minx; j < maxx; j++)
			{
				cv::Point p(j, i);
				if(face_index != -1 && /*last point is in triangle[face_index]*/
					PointInTriangle(m_tgtPoints[m_faces[face_index].x], m_tgtPoints[m_faces[face_index].y], m_tgtPoints[m_faces[face_index].z], p))
				{//guess next point is in the same triangle
					cv::Point q = FindMappedPoint(p, m_tgtPoints, m_srcPoints, face_index);
					m_tgtImage.at<Vec3b>(i, j) = m_srcImage.at<Vec3b>(q.y, q.x);
				}
				else//last point is outside the triangle or the current point is not in the last triangle
				{
					if((face_index = FindPointInFace(p, m_tgtPoints)) != -1)
					{
						cv::Point q = FindMappedPoint(p, m_tgtPoints, m_srcPoints, face_index);
						m_tgtImage.at<Vec3b>(i, j) = m_srcImage.at<Vec3b>(q.y, q.x);
					}
				}
			}
		}
	}
	
	std::vector<Face> m_faces;
	std::vector<cv::Point> m_srcPoints;//need to be warpped
	std::vector<cv::Point> m_tgtPoints;//warpped
	cv::Mat m_srcImage;
	cv::Mat m_tgtImage;
	int minx, maxx, miny, maxy;
};

#endif