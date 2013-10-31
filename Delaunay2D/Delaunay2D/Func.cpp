#include "Func.h"
//#include <Eigen/src/Core/Matrix.h>

using namespace Eigen;

void Triangulation::read_file(const char* filename)
{
	char str[256];
	int x, y, z;
	FILE* fp;
	fopen_s(&fp, filename, "r");
	while(fgets(str, 256, fp) != NULL)
	{
		if(str[0] == 'v')
		{
			sscanf_s(str+1, "%d%d%d", &x, &y, &z);
			point_list.push_back(cv::Point(x, y));
			attri_list.push_back(z == 0);//z=0 hard, z=1 soft
		}
		else if(str[0] == 'f')
		{
			sscanf_s(str+1, "%d%d%d", &x, &y, &z);
			face_list.push_back(Face(x, y, z));
		}
	}
	fclose(fp);
}

bool Triangulation::is_connect(int p1, int p2)
{
	for(int i = 0; i < face_list.size(); i++)
	{
		if(((face_list[i].x==p1)||(face_list[i].y==p1)||(face_list[i].z==p1))&&((face_list[i].x==p2)||(face_list[i].y==p2)||(face_list[i].z==p2)))
			return true;
	}
	return false;
}

void Triangulation::set_connect()
{
	mat_connect = new cv::Mat(height, width, CV_8UC4);
	cvZero(&mat_connect);
	for(int i = 0; i < point_list.size(); i++)
	{
		for(int j = i+1; j < point_list.size(); j++)
		{
			if(is_connect(i+1, j+1))
			{
				cv::Scalar s;
				s.val[0] = 1;
				cvSet2D(mat_connect, i, j, s);
				cvSet2D(mat_connect, j, i, s);
			}
		}
	}
}

void Triangulation::insert_points()
{
	for(int i = 0; i < point_list.size(); i++)
	{
		int x_P_length = x_back_Position.size();
		int y_P_length = y_back_Position.size();
		x_back_Position.insert(std::map<int, int>::value_type( x_back_Position.size(), point_list[i].x));
		y_back_Position.insert(std::map<int, int>::value_type( y_back_Position.size(), point_list[i].y));

		if(attri_list[i])//hard	
		{
			x_hard_IndexMat.insert(std::map<int, int>::value_type(x_hard_IndexMat.size(),x_P_length));
			y_hard_IndexMat.insert(std::map<int, int>::value_type(y_hard_IndexMat.size(),y_P_length));

			x_hard_MatIndex.insert(std::map<int, int>::value_type(x_P_length,x_hard_MatIndex.size()));
			y_hard_MatIndex.insert(std::map<int, int>::value_type(y_P_length,y_hard_MatIndex.size()));
		}
		else
		{
			x_soft_IndexMat.insert(std::map<int, int>::value_type(x_soft_IndexMat.size(),x_P_length));
			y_soft_IndexMat.insert(std::map<int, int>::value_type(y_soft_IndexMat.size(),y_P_length));
		
			x_soft_MatIndex.insert(std::map<int, int>::value_type(x_P_length,x_soft_MatIndex.size()));
			y_soft_MatIndex.insert(std::map<int, int>::value_type(y_P_length,y_soft_MatIndex.size()));
		}
	}
}

double Triangulation::cal_error(std::map<int, int> &cor_x,std::map<int, int> &cor_y)
{
	double cal_error = 0;
	for(int i = 0; i<cor_x.size(); i++)
	{
		for(int j = i+1; j<cor_x.size(); j++ )
		{			
			CvScalar con_flag = cvGet2D(mat_connect, i, j);
			if(x_hard_MatIndex.count(i) &&x_hard_MatIndex.count(j)&&y_hard_MatIndex.count(i) &&y_hard_MatIndex.count(j))
			{
				continue;
			}
			if(con_flag.val[0]>0.5)
			{
				int qix = cor_x[i];
				int qiy = cor_y[i];
				int qjx = cor_x[j];
				int qjy = cor_y[j];	
				int qix_back = x_back_Position[i];
				int qiy_back = y_back_Position[i];
				int qjx_back = x_back_Position[j];
				int qjy_back = y_back_Position[j];	
				double qij_dis = (qix-qjx)*(qix-qjx) + (qiy-qjy)*(qiy-qjy);
				double syij = 1;//(ColRatio[qix_back]+ColRatio[qjx_back])/2; 
				double sxij = 1;//(RowRatio[qiy_back]+RowRatio[qjy_back])/2;
				double lxij = x_back_Position[i]-x_back_Position[j];
				double lyij = y_back_Position[i]-y_back_Position[j];
				double lij = sxij*lxij*sxij*lxij+syij*lyij*syij*lyij;						
				if(lij<eps)
				{
					continue;
				}
				//calculate wij
				//find k1 k2
				int wij;
				int index_k1 = -1;
				int index_k2 = -1;
				for(int z=0;z<cor_x.size();z++)
				{
					CvScalar flag_k1 = cvGet2D(mat_connect,i,z);
					CvScalar flag_k2 = cvGet2D(mat_connect,j,z);
					if(flag_k1.val[0]*flag_k2.val[0]>0.5)
					{
						if(index_k1<0)
						{
							index_k1 = z;
						}
						else
						{
							index_k2 = z;
						}
					}

				}

				if((index_k1<0)||(index_k2<0)||(index_k1 == index_k2))
				{
					wij = 1;					
				}
				else
				{
					int k1_x = cor_x[index_k1];
					int k1_y = cor_y[index_k1];
					int k2_x = cor_x[index_k2];
					int k2_y = cor_y[index_k2];
					int qik1_x = k1_x-qix;
					int qik1_y = k1_y-qiy;
					int qjk1_x = k1_x-qjx;
					int qjk1_y = k1_y-qjy;
					int qik2_x = k2_x-qix;
					int qik2_y = k2_y-qiy;
					int qjk2_x = k2_x-qjx;
					int qjk2_y = k2_y-qjy;
					int signijk1q = qik1_x*qjk1_y-qik1_y*qjk1_x;
					int signijk2q = qik2_x*qjk2_y-qik2_y*qjk2_x;

					//original
					int pix = x_back_Position[i];
					int piy = y_back_Position[i];
					int pjx = x_back_Position[j];
					int pjy = y_back_Position[j];
					int m1_x = x_back_Position[index_k1];
					int m1_y = y_back_Position[index_k1];
					int m2_x = x_back_Position[index_k2];
					int m2_y = y_back_Position[index_k2];
					int pik1_x = m1_x-pix;
					int pik1_y = m1_y-piy;
					int pjk1_x = m1_x-pjx;
					int pjk1_y = m1_y-pjy;
					int pik2_x = m2_x-pix;
					int pik2_y = m2_y-piy;
					int pjk2_x = m2_x-pjx;
					int pjk2_y = m2_y-pjy;
					int signijk1p = pik1_x*pjk1_y-pik1_y*pjk1_x;
					int signijk2p = pik2_x*pjk2_y-pik2_y*pjk2_x;
					if((signijk1p*signijk1q<0)||(signijk2p*signijk2q<0))
					{
						wij = -1;
					}
					else
					{
						wij = 1;
					}
				}
				cal_error += (wij*qij_dis-lij)*(wij*qij_dis-lij)/lij; 	
			}//if connect judgy
		}
	}
	return cal_error;

}

void Triangulation::multi_grid_solver_newton()
{
	MatrixXd A_newton_x(x_soft_IndexMat.size(),x_soft_IndexMat.size());
	VectorXd B_newton_x(x_soft_IndexMat.size());
	A_newton_x.setConstant(0);
	B_newton_x.setConstant(0);

	MatrixXd A_newton_y(y_soft_IndexMat.size(),y_soft_IndexMat.size());
	VectorXd B_newton_y(y_soft_IndexMat.size());
	A_newton_y.setConstant(0);
	B_newton_y.setConstant(0);
	std::ofstream oStream;
	oStream.open("AllTheResult.txt");	
	//std::cout<<x_soft_IndexMat.size()<<std::endl;
	for(int i = 0; i<x_Position.size(); i++)
	{
		for(int j = i+1; j<x_Position.size(); j++ )
		{			
			
			CvScalar con_flag = cvGet2D(mat_connect, i, j);
			if(con_flag.val[0]>0.5)
			{
				/*if(i==78 && j==86)
				{
					oStream<<std::endl;
				}*/
				int qix = x_Position[i];
				int qiy = y_Position[i];
				int qjx = x_Position[j];
				int qjy = y_Position[j];	
				int qix_back = x_back_Position[i];
				int qiy_back = y_back_Position[i];
				int qjx_back = x_back_Position[j];
				int qjy_back = y_back_Position[j];	
				double qij_dis = (qix-qjx)*(qix-qjx) + (qiy-qjy)*(qiy-qjy);
				double syij = 1;//(ColRatio[qix_back]+ColRatio[qjx_back])/2; 
				double sxij = 1;//(RowRatio[qiy_back]+RowRatio[qjy_back])/2;
				double lxij = x_back_Position[i]-x_back_Position[j];
				double lyij = y_back_Position[i]-y_back_Position[j];
				double lij = sxij*lxij*sxij*lxij+syij*lyij*syij*lyij;						
				if(lij<eps)
				{
					continue;
				}
				//calculate wij
				//find k1 k2
				int wij;
				int index_k1 = -1;
				int index_k2 = -1;
				for(int z=0;z<x_Position.size();z++)
				{
					CvScalar flag_k1 = cvGet2D(mat_connect,i,z);
					CvScalar flag_k2 = cvGet2D(mat_connect,j,z);
					if(flag_k1.val[0]*flag_k2.val[0]>0.5)
					{
						if(index_k1<0)
						{
							index_k1 = z;
						}
						else
						{
							index_k2 = z;
						}
					}

				}
				if((index_k1<0)||(index_k2<0)||(index_k1 == index_k2))
				{
					wij = 1;					
				}
				else
				{
					int k1_x = x_Position[index_k1];
					int k1_y = y_Position[index_k1];
					int k2_x = x_Position[index_k2];
					int k2_y = y_Position[index_k2];
					int qik1_x = k1_x-qix;
					int qik1_y = k1_y-qiy;
					int qjk1_x = k1_x-qjx;
					int qjk1_y = k1_y-qjy;
					int qik2_x = k2_x-qix;
					int qik2_y = k2_y-qiy;
					int qjk2_x = k2_x-qjx;
					int qjk2_y = k2_y-qjy;
					int signijk1q = qik1_x*qjk1_y-qik1_y*qjk1_x;
					int signijk2q = qik2_x*qjk2_y-qik2_y*qjk2_x;

					//original
					int pix = x_back_Position[i];
					int piy = y_back_Position[i];
					int pjx = x_back_Position[j];
					int pjy = y_back_Position[j];
					int m1_x = x_back_Position[index_k1];
					int m1_y = y_back_Position[index_k1];
					int m2_x = x_back_Position[index_k2];
					int m2_y = y_back_Position[index_k2];
					int pik1_x = m1_x-pix;
					int pik1_y = m1_y-piy;
					int pjk1_x = m1_x-pjx;
					int pjk1_y = m1_y-pjy;
					int pik2_x = m2_x-pix;
					int pik2_y = m2_y-piy;
					int pjk2_x = m2_x-pjx;
					int pjk2_y = m2_y-pjy;
					int signijk1p = pik1_x*pjk1_y-pik1_y*pjk1_x;
					int signijk2p = pik2_x*pjk2_y-pik2_y*pjk2_x;
					if((signijk1p*signijk1q<0)||(signijk2p*signijk2q<0))
					{
						wij = -1;
					}
					else
					{
						wij = 1;
					}

				}											
				/*if(wij<0)
				{
					oStream<<i<<" "<<j<<" "<<std::endl;
				}*/
				//wij = 1;
				//error_x += (wij*qij_dis-lij)*(wij*qij_dis-lij)/lij;
				double magnify = 8;
				double bound_mag = 1;
				if(x_soft_MatIndex.count(i))
				{
					int i_n = x_soft_MatIndex[i];
					int j_n; 
					if(x_soft_MatIndex.count(j))
					{
						j_n = x_soft_MatIndex[j];
						/*if(i_n ==4 || j_n ==4)
						{
							oStream<<i<<" "<<j<<" "<<std::endl;
						}*/
						double temp = (wij*qij_dis-lij)/lij;
						
						A_newton_x(i_n,i_n) += magnify*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij; 
						A_newton_x(i_n, j_n) += -magnify*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij;
						B_newton_x(i_n) += -(wij*qij_dis-lij)*wij*(qix-qjx)/lij;
						
						A_newton_x(j_n,j_n) += magnify*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij; 
						A_newton_x(j_n, i_n) += -magnify*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij;
						B_newton_x(j_n) += -(wij*qij_dis-lij)*wij*(qjx-qix)/lij;
							
									
					}
					else
					{
						//if(i_n ==4)
						//{
						//	double temp = bound_mag*magnify*qjx*(wij*qij_dis-lij)/lij;
						//	/*oStream<<i<<" "<<j<<" "<<std::endl;*/
						//}
						B_newton_x(i_n) += -bound_mag*wij*(wij*qij_dis-lij)*(qix-qjx)/lij;
						//B_newton_x(i_n) += bound_mag*magnify*qjx*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij;
						A_newton_x(i_n,i_n) += bound_mag*magnify*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij; 
					}
				}
				else
				{
					if(x_soft_MatIndex.count(j))
					{
						int j_n = x_soft_MatIndex[j];
						if(j_n ==4)
						{
							double temp = bound_mag*magnify*qix*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij;
							/*oStream<<i<<" "<<j<<" "<<std::endl;*/
						}
						B_newton_x(j_n) += -bound_mag*wij*(wij*qij_dis-lij)*(qjx-qix)/lij;
						//B_newton_x(j_n) += bound_mag*magnify*wij*(wij*qij_dis-lij)*(qjx-qix)/lij;
						//B_newton_x(j_n) += bound_mag*magnify*qix*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij;
						A_newton_x(j_n,j_n) += bound_mag*magnify*(qij_dis-wij*lij+2*(qix-qjx)*(qix-qjx))/lij; 
					}
				}//for x_soft_index

				if(y_soft_MatIndex.count(i))
				{
					int i_n = y_soft_MatIndex[i];
					int j_n; 
					if(y_soft_MatIndex.count(j))
					{
						j_n = y_soft_MatIndex[j]; 
						A_newton_y(i_n,i_n) += magnify*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij; 	
						A_newton_y(i_n,j_n) += magnify*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij;
						B_newton_y(i_n) += -(wij*qij_dis-lij)*wij*(qiy-qjy)/lij;

						A_newton_y(j_n,j_n) += magnify*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij; 
						A_newton_y(j_n,i_n) += magnify*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij;
						B_newton_y(j_n) += -(wij*qij_dis-lij)*wij*(qjy-qiy)/lij;
						
									
					}
					else
					{						
						B_newton_y(i_n) += -bound_mag*wij*(wij*qij_dis-lij)*(qiy-qjy)/lij;
						//B_newton_y(i_n) += bound_mag*magnify*qjy*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij;
						A_newton_y(i_n,i_n) += bound_mag*magnify*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij; 
					}
				}
				else
				{
					if(y_soft_MatIndex.count(j))
					{
						int j_n = y_soft_MatIndex[j];
						B_newton_y(j_n) += -bound_mag*wij*(wij*qij_dis-lij)*(qjy-qiy)/lij;
						//B_newton_y(j_n) += bound_mag*magnify*qjy*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij;
						A_newton_y(j_n,j_n) += bound_mag*magnify*(qij_dis-wij*lij+2*(qiy-qjy)*(qiy-qjy))/lij; 
					}
				}//for y_soft_MatIndex
				
			}//for connect judge
			//oStream<<i<<" "<<j<<" "<<std::endl;
		}//for j
		
	}//for i
	
	/*for(int i = 0; i<B_newton_x.size(); i++)
	{
		error_x += -B_newton_x 
	}*/
	/*for(int i = 0; i<B_newton_x.size(); i++)
	{
		if(std::abs(A_newton_x(i,i))<eps)
		{
			A_newton_x(i,i) = 1;
			B_newton_x(i) = x_Position[x_soft_IndexMat[i]];
		}
	}
	for(int i = 0; i<B_newton_y.size(); i++)
	{
		if(std::abs(A_newton_y(i,i)<eps))
		{
			A_newton_y(i,i) = 1;
			B_newton_y(i) = y_Position[y_soft_IndexMat[i]];
		}
	}*/
	//double temp_55 = A_newton_x(5,5);
	/*if(std::abs(temp_55)>0.1)
	{
			oStream<<std::endl;
	}*/
	//std::cout<<x_soft_IndexMat.size()<<std::endl;
	//too slow
	//VectorXd S_newton_x = A_newton_x.jacobiSvd(ComputeThinU | ComputeThinV).solve(B_newton_x);
	//VectorXd S_newton_y = A_newton_y.jacobiSvd(ComputeThinU | ComputeThinV).solve(B_newton_y);
	VectorXd S_newton_x = A_newton_x.lu().solve(B_newton_x);
	VectorXd S_newton_y = A_newton_y.lu().solve(B_newton_y);
	oStream<<"S_newton_x= "<<std::endl<<S_newton_x<<std::endl;
	oStream<<"A_newton_x= "<<std::endl<<A_newton_x<<std::endl;
	oStream<<"B_newton_x= "<<std::endl<<B_newton_x<<std::endl;
	oStream<<"differ"<<std::endl;
	for(int i=0;i<S_newton_x.size(); i++)
	{		
		oStream<<x_Position[x_soft_IndexMat[i]]-S_newton_x(i)<<std::endl;
	}
	oStream<<"S_newton_y= "<<std::endl<<S_newton_y<<std::endl;
	oStream<<"A_newton_y= "<<std::endl<<A_newton_y<<std::endl;
	oStream<<"B_newton_y= "<<std::endl<<B_newton_y<<std::endl;
	//oStream<<"B_newton_x "<<std::endl<<B_newton_x<<std::endl;

	std::map<int, int> x_Position_temp;
	std::map<int, int> y_Position_temp;
	for(int i = 0; i<x_Position.size(); i++)
	{
		x_Position_temp[i] = x_Position[i];
				
	}
	for(int i = 0; i<y_Position.size(); i++)
	{
		y_Position_temp[i] = y_Position[i];
				
	}
	for(int i = 0; i<B_newton_x.size(); i++)
	{
		x_Position_temp[x_soft_IndexMat[i]] += S_newton_x(i);
				
	}
	for(int i = 0; i<B_newton_y.size(); i++)
	{
		
		y_Position_temp[y_soft_IndexMat[i]] += S_newton_y(i);			
		
	}
	if(iter_count ==0)
	{
		//error = 99999999;
	}
	double error_new = cal_error(x_Position_temp, y_Position_temp);
	std::cout<<"error "<<iter_count<<" "<<error_new<<std::endl;
	if(error-error_new>eps)
	{
		error = error_new;
		for(int i = 0; i<B_newton_x.size(); i++)
		{
			x_Position[x_soft_IndexMat[i]] += S_newton_x(i);
			y_Position[y_soft_IndexMat[i]] += S_newton_y(i);				
		}

	}
	else
	{
		iter_flag = iter_count -1;
	}
	/*SelfAdjointEigenSolver<MatrixXd> eigensolver(A_newton_x);*/
	//if (eigensolver.info() != Success) abort();
	//oStream << "The eigenvalues of m are:\n" << eigensolver.eigenvalues() <<std::endl;
	//oStream<< "Here's a matrix whose columns are eigenvectors of y \n"<<std::endl;
	////<< "corresponding to these eigenvalues:\n"
	////<< eigensolver.eigenvectors() << endl;
	//oStream<<"min"<<std::endl; 
	//oStream<< eigensolver.eigenvalues()[0]<<std::endl;

}

void Triangulation::run()
{
	iter_flag = -1;
	iter_count = 0;
	error = cal_error(x_Position, y_Position);
	std::cout<<"error initial "<<error<<std::endl;
	while(iter_flag<-0.5)
	{
		multi_grid_solver_newton();
		iter_count++;		
	}
	std::cout<<"iter times"<<iter_count<<std::endl;
	std::cout<<"iter used"<<iter_flag<<std::endl;
}

void Triangulation::prerun()
{
	set_connect();
	insert_points();
}