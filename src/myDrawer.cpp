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
#include <nav_msgs/Path.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

using namespace cv;
using namespace std;

int drawpositions=500;

typedef struct DrawData{
	vector<Point> positions;
	vector<Point> particles;
	vector<Point> laserScan;
}DrawData;

typedef struct DrawConfig{
	float resolution;
	Point2f origin;
	bool showparticles;
	bool showPath;
	DrawConfig() :
		resolution(0.05), origin(Point2f(0.0000, 0.0000)),  showparticles(false), showPath(true){};
}DrawConfig;

struct DrawData drawData;
struct DrawConfig drawConfig;
Mat mapa, show;



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

Point toimage(float x, float y){
	static Point maporigin=Point(drawConfig.origin.x/drawConfig.resolution, mapa.rows+drawConfig.origin.y/drawConfig.resolution);
	Point p;
	p.x = x/drawConfig.resolution - maporigin.x;
	p.y = maporigin.y - y/drawConfig.resolution;
	return p;	
	}

Point2f tomap(int x, int y){
	static Point maporigin=Point(drawConfig.origin.x/drawConfig.resolution, mapa.rows+drawConfig.origin.y/drawConfig.resolution);
	Point2f p;
	p.x=(x+maporigin.x)*drawConfig.resolution;
	p.y=(maporigin.y-y)*drawConfig.resolution;
	return p;
	}


Point3f poseActual;
float disttravelled=0.0;
Point3f startPoint=Point3f(0,0,0);
void LocCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    if(msg->pose.pose.position.x==0 && msg->pose.pose.position.y==0) return;
	poseActual=Point3f(msg->pose.pose.position.x,msg->pose.pose.position.y,tf::getYaw(msg->pose.pose.orientation));
	drawData.positions.push_back(toimage(msg->pose.pose.position.x, msg->pose.pose.position.y));
    float dx=poseActual.x-startPoint.x;
    float dy=poseActual.y-startPoint.y;
    disttravelled=disttravelled+sqrt(dx*dx+dy*dy);
    printf("Dist travelled=%f (+%f)\n ",disttravelled,sqrt(dx*dx+dy*dy));
    startPoint=poseActual;
	}

void AmclPartCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
	drawData.particles.clear();
	for (unsigned int i = 0; i < msg->poses.size(); i++){
		Point p1=toimage(msg->poses[i].position.x, msg->poses[i].position.y);
		float ang=tf::getYaw(msg->poses[i].orientation);
		Point p2=Point(p1.x+4*cos(-ang), p1.y+4*sin(-ang));
		drawData.particles.push_back(p1);
		drawData.particles.push_back(p2);
		}

	}

void ScanCallback (const sensor_msgs::LaserScan::ConstPtr& msg){
	static tf::TransformListener listener;
	static laser_geometry::LaserProjection projector;
	
 	try{
		sensor_msgs::PointCloud cloud;
		projector.transformLaserScanToPointCloud("map",*msg, cloud, listener);
	  	// Do something with cloud.
		drawData.laserScan.clear();
		for(unsigned int i=0; i<cloud.points.size(); i=i+4) drawData.laserScan.push_back(toimage(cloud.points[i].x,cloud.points[i].y));
    	}
    catch (tf::TransformException ex){
      	ROS_ERROR("%s",ex.what());
    	}
	}

bool recording=false;
bool reproducing=false;
int currentstep=-1;
vector<Point> route;
int pathwidth=1;
bool changecolour=false;
int step=2;
void drawGUI(){
    static int currentcolour=0;
    static Scalar colours[5]={CV_RGB(255,50,50), CV_RGB(0,255,255), CV_RGB(127,0,255), CV_RGB(125,0,255), CV_RGB(50,125,0) };
    if(changecolour){ currentcolour=(currentcolour+1)%5; changecolour=false;}
    resize(mapa, show, Size(), 2.0, 2.0);
	//draw part
	if(drawConfig.showparticles)
		for(unsigned int i = 0; i<drawData.particles.size(); i=i+2)
			line(show, drawData.particles[i]*2, drawData.particles[i+1], CV_RGB(100,100,255),1,CV_AA);
	//draw historical robot path
    int positionsdrawn=0;
	if(drawConfig.showPath && drawData.positions.size()>2){
		for(unsigned int i = (drawData.positions.size()-1); i>0 ; i=i-step){
            if(i<(drawData.positions.size()-2)) 
                line(show, drawData.positions[i]*2, drawData.positions[i-1]*2, colours[currentcolour],pathwidth,CV_AA);
            positionsdrawn++;
            if(positionsdrawn>drawpositions) break;
            }
        }

    //draw received route
	for(unsigned int i =0; i<route.size(); i++){
        if(i>0){
            //line(show, route[i]*2, route[i-1]*2, CV_RGB(200-200/route.size()*i, 200-200/route.size()*i,255),pathwidth/2+1,CV_AA);
            circle(show, route[i-1]*2, pathwidth*2, CV_RGB(100,100,255), 2, CV_AA);
            }
        }
    if(route.size()>0) circle(show, route[route.size()-1]*2, pathwidth*2, CV_RGB(100,100,255),  2, CV_AA);

	//draw robot pose
	if(drawData.positions.size()){
		Scalar colour=CV_RGB(255,0,255);
		circle(show, drawData.positions.back()*2, 5*2, colour,-1,CV_AA);
		Point pt=Point(drawData.positions.back().x+5*cos(-poseActual.z), drawData.positions.back().y+6*sin(-poseActual.z));
		line(show, drawData.positions.back()*2, pt*2, CV_RGB(255,255,255),1,CV_AA);
		}
	//draw laser scans
	for(unsigned int i = 0; i<drawData.laserScan.size(); i++)
		circle(show, drawData.laserScan[i]*2, 1, CV_RGB(0,0,255), -1, 2, 0);
	}



void msgToMat(const nav_msgs::OccupancyGrid::ConstPtr grid, Mat &image){
    image = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC3);
	for(int j = grid->info.height - 1; j>= 0; j--){
		uchar* data=image.ptr<uchar>(j);
		for(unsigned int i = 0; i < grid->info.width; i++){
            int grid_row = grid->info.height - 1 - j;
            int grid_data = grid->data[grid_row * grid->info.width + i];
            int value=0;
            if (grid_data != -1){
               value= 255 - (255 * grid_data) / 100;
                }
            else{
               value= 128;
                }
			int pixel=i*3;
			data[pixel]=value;
			data[pixel+1]=value;
			data[pixel+2]=value;
			}
		}
    }



bool mapreceived=false;
void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    cv::Mat grid;
    Mat img;
    msgToMat(msg, mapa);
    printf("Received MapData!!!!\n");
    mapreceived=true;
    }


void RouteCallback(const nav_msgs::Path::ConstPtr& msg){
    route.clear();
    for(unsigned int i=0; i<msg->poses.size(); i++){
        route.push_back( toimage( msg->poses[i].pose.position.x, msg->poses[i].pose.position.y ) );
        }
    }

int main(int argc, char **argv){
	ros::init(argc, argv, "sendgoal");
	static ros::NodeHandle n;
	static ros::Subscriber sub = n.subscribe("/pose", 2, LocCallback);
	static ros::Subscriber sub2 = n.subscribe("/particlecloud", 2, AmclPartCallback);
    static ros::Subscriber sub3 = n.subscribe("/map", 1000, MapCallback);
    static ros::Subscriber sub4 = n.subscribe("/currentroute", 1000, RouteCallback);
	namedWindow("map", CV_GUI_EXPANDED);
	
    while(!mapreceived) { ros::spinOnce(); printf("Waiting for map!\n"); msleep(250); }
    printf("Map received!");
    
	for(;;){
		ros::spinOnce();
		drawGUI();
		imshow("map", show);
		char key = (char) cvWaitKey(1);
		if(key=='r'){
			disttravelled=0.0;
            startPoint=poseActual;
			}
		if( key=='o'){ 
			drawConfig.showPath=!drawConfig.showPath; 
			if(drawConfig.showparticles) printf("Showing path: YES\n"); else printf("Showing path: NO\n"); 
			}
		if( key=='p'){ 
			drawConfig.showparticles=!drawConfig.showparticles; 
			if(drawConfig.showparticles) printf("Showing particles: YES\n"); else printf("Showing particles: NO\n"); 
			}
		if( key=='+'){ 
			drawpositions=drawpositions+3;
            printf("Going to draw %d positions\n",drawpositions);
			}
		if( key=='-'){ 
			drawpositions=drawpositions-3;
            if(drawpositions<10) drawpositions=10;
            printf("Going to draw %d positions\n",drawpositions);
			}
		if( key=='w'){ 
			pathwidth++;
            printf("Path width=%d\n",pathwidth);
			}
		if( key=='s'){ 
			pathwidth--;
            if(pathwidth<1) pathwidth=1;
            printf("Path width=%d\n",pathwidth);
			}
        if( key == 'd'){
            changecolour=true;
            }
		if( key=='t'){ 
			step++;
            printf("step=%d\n",step);
			}
		if( key=='g'){ 
			step--;
            if(step<1) step=1;
            printf("step=%d\n",step);
			}
		if( key == 'q' || key == 27 ) break;
		}

  return 0;
}
