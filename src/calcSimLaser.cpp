#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
using namespace cv;
using namespace std;


//********************		
// Tic and toc matlab-like functions
//********************
struct timeval starttime;
void tic(){
	gettimeofday(&starttime,NULL);
	}
double toc(){
	struct timeval rightnow;
	gettimeofday(&rightnow,NULL);
	double timeinms=((rightnow.tv_sec-starttime.tv_sec)*1000000+(rightnow.tv_usec-starttime.tv_usec))/1000;
	return timeinms;
	}

//********************
// msleep
//********************
int __nsleep(const struct timespec *req, struct timespec *rem)
	{
	    struct timespec temp_rem;
	    if(nanosleep(req,rem)==-1)
	        return __nsleep(rem,&temp_rem);
	    else
	        return 1;
	}

int msleep(unsigned long milisec)
{
	    struct timespec req,rem;
	    time_t sec=(int)(milisec/1000);
	    milisec=milisec-(sec*1000);
	    req.tv_sec=sec;
	    req.tv_nsec=milisec*1000000L;
	    __nsleep(&req,&rem);
	    return 1;
}









void createDistanceMatrix(Mat image){
	tic();
	FileStorage fs;
	printf("Creating data file...\n");
	//we need to compute it
	static float max_range=31.0/0.05;
	vector<Point> obstacles;
    vector<Point> freespace;
	for(int j=0; j<(image.rows-1); j++){
		uchar* data=image.ptr<uchar>(j);
		uchar* nextrow=image.ptr<uchar>(j+1);
		for(int i=0; i<(image.cols-1); i++){
			if(data[i]>250){
				freespace.push_back(Point(i,j));
				if(nextrow[i]>250) nextrow[i]=200;
				if(nextrow[i+1]>250) nextrow[i+1]=200;
				if(data[i+1]>250) data[i+1]=200;
				}
			else if(data[i]<10) obstacles.push_back(Point(i,j));
			}
		}
	Mat distances = cv::Mat::zeros(freespace.size(), 360, CV_32FC1)+max_range;
	for(unsigned int j=0; j<freespace.size(); j++){ //para cada espacio libre
		if(j%1000==0) printf("Calculated %d out of %d points\n",j,(int)freespace.size());
		float* distancehist=distances.ptr<float>(j);
		for(unsigned int i=0; i<obstacles.size(); i++){ //para cada obstáculo
			//computamos la distancia punto a punto
			float y=obstacles[i].y-freespace[j].y;
			float x=obstacles[i].x-freespace[j].x;
			float dcentro=sqrt(y*y+x*x);

			int grados[4];
			grados[0]=cvRound(atan2(-y-0.5, x-0.5)*180.0/CV_PI);
			if(grados[0]<0)  grados[0]=360+grados[0];
			grados[1]=cvRound(atan2(-y-0.5, x+0.5)*180.0/CV_PI);
			grados[2]=cvRound(atan2(-y+0.5, x+0.5)*180.0/CV_PI);
			grados[3]=cvRound(atan2(-y+0.5, x-0.5)*180.0/CV_PI);

			int angmin=grados[0];
			int angmax=grados[0];
			for(int corner=1; corner<4; corner++){
				if(grados[corner]<0) grados[corner]=360+grados[corner];
				angmin=min(grados[corner],angmin);
				angmax=max(grados[corner],angmax);
				}
			if((angmax-angmin)>100){
				for(int ang=angmax; ang<=(360+angmin); ang++){
					if(distancehist[ang%360]>dcentro) distancehist[ang%360]=dcentro;
					}
				}
			else{
				for(int ang=angmin; ang<=angmax; ang++){
					if(distancehist[ang]>dcentro) distancehist[ang]=dcentro;
					}
				}
			}
		}
	printf("Saving file to disk...\n");
	fs.open("distances.yml", FileStorage::WRITE);
	fs << "distances" << distances;
	fs << "freespace" << freespace;
	fs.release();
	cout << "Total time:" << toc()/1000 << endl;
	}




void msgToMat(const nav_msgs::OccupancyGrid::ConstPtr grid, Mat &image){
    image = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);
    for (int row = grid->info.height - 1; row >= 0; row--) {
            for (unsigned int col = 0; col < grid->info.width; col++) {
                int grid_row = grid->info.height - 1 - row;
                int grid_data = grid->data[grid_row * grid->info.width + col];
                if (grid_data != -1){
                   image.at<uchar>(row, col)= 255 - (255 * grid_data) / 100;
                    }
                else{
                   image.at<uchar>(row, col)= 128;
                    }
            }
        }
    }



//map image (BN)
Mat mapa;
bool mapreceived=false;
void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    cv::Mat grid;
    msgToMat(msg, mapa);
    printf("Received MapData!!!!\n");
    mapreceived=true;
    }


int main(int argc, char **argv){
    ros::init(argc, argv, "sendgoal");
	static ros::NodeHandle n;
    static ros::Subscriber sub = n.subscribe("/map", 1000, MapCallback);
    printf("Waiting for map ...");
    while(!mapreceived) { ros::spinOnce(); printf("."); fflush(stdout); msleep(250); }
    imwrite("recivedimg.jpg",mapa);
    createDistanceMatrix(mapa);
    return 0;
    }
