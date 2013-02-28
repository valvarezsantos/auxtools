#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

using namespace cv;
using namespace std;



typedef struct DrawData{
	vector<Point> positions;
	vector<Point> particles;
	vector<Point> laserScan;
	vector<Point> fakelaserScan;
	Point2f goalorigin, poseorigin;
	Point2f goalfinish, posefinish;
}DrawData;

typedef struct DrawConfig{
	float resolution;
	Point2f origin;
	bool showparticles;
	bool showPath;
	bool newpos;
	bool newparts;
	unsigned int pathlength;
	int showposeinitializer;
	int showgoal;
	DrawConfig() :
		resolution(0.02500), origin(Point2f(-40.0000, -17.0000)),  showparticles(true), showPath(false), newpos(false), newparts(false), pathlength(500), showposeinitializer(0), showgoal(0){};
} DrawConfig;

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

//AdrianC
void parseYamlMap(std::string path, std::string fileName){
	FileStorage fs;
	fs.open(path + "/" + fileName, FileStorage::READ);

	std::string mapImagePath = fs["image"];
	drawConfig.resolution = (float) fs["resolution"];
	std::vector<float> origin;
	fs["origin"] >> origin;
	mapa = imread(path + "/" + mapImagePath);
	mapa.copyTo(show);
	drawConfig.origin.x = origin[0];
	drawConfig.origin.y = origin[1];

	std::cout << "MapImage: " << mapImagePath << std::endl;
	std::cout << "MapRes: " << mapa.rows << "x" << mapa.cols << std::endl;
	std::cout << "Resolution: " << drawConfig.resolution << std::endl;
	std::cout << "Origin: (" << drawConfig.origin.x << ", " << drawConfig.origin.y << ")" << std::endl;
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

Mat distances;
Point3f poseActual;
float likelihood;
vector<Point> freespace;
float histlaser[270];
Mat h1;
Mat h2;
void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	poseActual=Point3f(msg->pose.pose.position.x,msg->pose.pose.position.y,tf::getYaw(msg->pose.pose.orientation));
	drawData.positions.push_back(toimage(msg->pose.pose.position.x, msg->pose.pose.position.y));
	if(drawData.positions.size()>drawConfig.pathlength) drawData.positions.erase (drawData.positions.begin());
	tic();
	for(unsigned int i=0; i<freespace.size(); i++){
		int vx=freespace[i].x-drawData.positions.back().x;
		int vy=freespace[i].y-drawData.positions.back().y;
		if(abs(vx)<2 && abs(vy)<2){
			float histsimulado[270];
			float* data=distances.ptr<float>(i);
			int angrobot=poseActual.z*180.0/CV_PI;
			int startangle=cvRound(angrobot-135+360)%360;
			int endangle=cvRound(angrobot+135+360);
			if(endangle-startangle>270) endangle=endangle%360;
			int k=0;
			drawData.fakelaserScan.clear();
			for(int j=startangle; j<endangle; j++){
				histsimulado[k++]=data[(j%360)]*drawConfig.resolution;
				drawData.fakelaserScan.push_back(freespace[i]+Point(cvRound(cos((j%360)*CV_PI/180.0)*data[(j%360)]), cvRound(-sin((j%360)*CV_PI/180.0)*data[(j%360)])) );
				}
			h1 = Mat(1, 270, CV_32F, histlaser).clone();
			h2 = Mat(1, 270, CV_32F, histsimulado).clone();
			//cout << h1 << endl;
			//cout << h2 << endl;
			likelihood=compareHist(h1, h2, CV_COMP_BHATTACHARYYA);
			float distmedia=0.0;
			for(int j=0; j<270; j++){
				distmedia+=abs(histlaser[j]-histsimulado[j]);
				}
			distmedia=distmedia/270;
			printf("distmedia=%f\n",distmedia);
			printf("likelihood=%f\n",likelihood);
			break;
			}
		}
	cout << "Tiempo comparacion: " << toc() << endl;
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
	for(int i=0; i<1080; i=i+4){
		histlaser[(int)floor(i/4)]=msg->ranges[i];
		if(histlaser[i/4]>msg->range_max || histlaser[i/4]<msg->range_min) histlaser[i/4]=0.0;
		}
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

bool ordercompleted=false;
bool ordercancelled=false;
bool cancel=false;
void PathCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	static ros::NodeHandle n;
	static ros::Publisher cancel_goal_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

	bool allcompleted=true;
	bool anycancelled=false;
	printf("-------\n");
	for(unsigned int i=0; i<msg->status_list.size(); i++){
		printf("Order id: %s\n",msg->status_list[i].goal_id.id.c_str());
		printf("msg->status_list[%d].status=%d\n",i,msg->status_list[i].status);
		if(cancel){
			printf("Sending cancel msg...\n");
			cancel_goal_pub.publish(msg->status_list[i].goal_id);
			}
		if(msg->status_list[i].status==1){
			allcompleted=false;
			}
		if(msg->status_list[i].status==4){
			anycancelled=true;
			}
		}
	cancel=false;
	if(anycancelled) ordercancelled=true;
	else ordercancelled=false;
	if(allcompleted) ordercompleted=true;
	else ordercompleted=false;
	}

Point centrocrop;
static void onMouse(int event, int x, int y, int, void* ptr){
	static bool searchforfinish=false;
	static bool searchforfinish2=false;
	static ros::NodeHandle n;
	static ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	static ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    
	if( event == CV_EVENT_LBUTTONDOWN){
		drawData.goalorigin=Point(x,y);
		drawData.goalfinish=Point(x,y);
		searchforfinish=true;
		drawConfig.showgoal=0;
		}
	if( event == CV_EVENT_MOUSEMOVE && searchforfinish){
		drawConfig.showgoal=500;
		float vx=x-drawData.goalorigin.x;
		float vy=y-drawData.goalorigin.y;
		float modulo=sqrt(vx*vx+vy*vy)*drawConfig.resolution;
		drawData.goalfinish=Point(drawData.goalorigin.x+vx/modulo, drawData.goalorigin.y+vy/modulo);
		}
	if( event == CV_EVENT_LBUTTONUP ){
		searchforfinish=false;
		float vx=x-drawData.goalorigin.x;
		float vy=y-drawData.goalorigin.y;
		float modulo=sqrt(vx*vx+vy*vy)*drawConfig.resolution;
		if(modulo>0){
			drawData.goalfinish=Point(drawData.goalorigin.x+vx/modulo, drawData.goalorigin.y+vy/modulo);
			geometry_msgs::PoseStamped msg;
			//we'll send a goal to the robot
			msg.header.frame_id = "/map";
			msg.header.stamp = ros::Time::now();
			Point2f position=tomap(drawData.goalorigin.x, drawData.goalorigin.y);
			msg.pose.position.x=position.x;
			msg.pose.position.y=position.y;
			float yaw=atan2(-(drawData.goalfinish.y-drawData.goalorigin.y), drawData.goalfinish.x-drawData.goalorigin.x);
			msg.pose.orientation=tf::createQuaternionMsgFromYaw(yaw);
			ROS_INFO("Sending goal");
			goal_pub.publish(msg);
			}
		else{drawConfig.showgoal=0;}
		}
	
	if( event == CV_EVENT_RBUTTONDOWN){
		drawData.poseorigin=Point(x,y);
		drawData.posefinish=Point(x,y);
		searchforfinish2=true;
		drawConfig.showposeinitializer=0;
		}
	if( event == CV_EVENT_MOUSEMOVE && searchforfinish2){
		drawConfig.showposeinitializer=500;
		float vx=x-drawData.poseorigin.x;
		float vy=y-drawData.poseorigin.y;
		float modulo=sqrt(vx*vx+vy*vy)*drawConfig.resolution;
		drawData.posefinish=Point(drawData.poseorigin.x+vx/modulo, drawData.poseorigin.y+vy/modulo);
		}
	if( event == CV_EVENT_RBUTTONUP ){
		searchforfinish2=false;
		float vx=x-drawData.poseorigin.x;
		float vy=y-drawData.poseorigin.y;
		float modulo=sqrt(vx*vx+vy*vy)*drawConfig.resolution;
		if(modulo>0){
			drawData.posefinish=Point(drawData.poseorigin.x+vx/modulo, drawData.poseorigin.y+vy/modulo);
			geometry_msgs::PoseWithCovarianceStamped msg;
			//we'll send a pose to the robot
			msg.header.frame_id = "/map";
			msg.header.stamp = ros::Time::now();
			Point2f position=tomap(drawData.poseorigin.x, drawData.poseorigin.y);
			msg.pose.pose.position.x=position.x;
			msg.pose.pose.position.y=position.y;
			float yaw=atan2(-(drawData.posefinish.y-drawData.poseorigin.y), drawData.posefinish.x-drawData.poseorigin.x);
			msg.pose.pose.orientation=tf::createQuaternionMsgFromYaw(yaw);
			for(int i=0; i<36; i++) msg.pose.covariance[i]=0.0;
			msg.pose.covariance[0]=1.0;
			msg.pose.covariance[7]=1.0;
			msg.pose.covariance[21]=6.3; //45grados de cov
		//	0  1  2  3  4  5
		//	6  7  8  9  10 11
		//	12 13 14 15 16 17
 		//	18 19 20 21 22 23

			ROS_INFO("Sending initial pose");
			pose_pub.publish(msg);
			}
		else{drawConfig.showposeinitializer=0;}
		}
	}

bool recording=false;
bool reproducing=false;
int currentstep=-1;
vector<Point3f> route;
void drawGUI(){
	mapa.copyTo(show);
	//draw part
	if(drawConfig.showparticles)
		for(unsigned int i = 0; i<drawData.particles.size(); i=i+2)
			line(show, drawData.particles[i], drawData.particles[i+1], CV_RGB(100,100,255),1,CV_AA);
	//draw historical robot path
	if(drawConfig.showPath)
		for(unsigned int i = 0; i<drawData.positions.size(); i++)
			circle(show, drawData.positions[i], 3, CV_RGB(255,0,0), -1, 1, 0);
	//draw robot pose
	if(drawData.positions.size()){
		Scalar colour=CV_RGB(255,0,255);
		if(ordercompleted) colour=CV_RGB(0,255,0);
		if(ordercancelled) colour=CV_RGB(255,0,0);
		circle(show, drawData.positions.back(), 5, colour,-1,CV_AA);
		Point pt=Point(drawData.positions.back().x+5*cos(-poseActual.z), drawData.positions.back().y+6*sin(-poseActual.z));
		line(show, drawData.positions.back(), pt, CV_RGB(255,255,255),1,CV_AA);
		}
	//draw laser scans
	for(unsigned int i = 0; i<drawData.laserScan.size(); i++)
		circle(show, drawData.laserScan[i], 1, CV_RGB(0,0,255), -1, 2, 0);
	for(unsigned int i = 0; i<drawData.fakelaserScan.size(); i++)
		circle(show, drawData.fakelaserScan[i], 1, CV_RGB(255,0,255), -1, 2, 0);
	//draw goal and pose initializer
	if(drawConfig.showgoal>0){ line(show, drawData.goalorigin, drawData.goalfinish, CV_RGB(100,255,100), 3, 8, 0); drawConfig.showgoal--;}
	if(drawConfig.showposeinitializer>0){ line(show, drawData.poseorigin, drawData.posefinish, CV_RGB(255,100,100), 3, 8, 0); drawConfig.showposeinitializer--;}
	
	//draw route
	if(reproducing || recording){
		for(int i = 0; i<((int)route.size()-1); i++){
			line(show, toimage(route[i].x, route[i].y), toimage(route[i+1].x, route[i+1].y), CV_RGB(255,128,255), 2,CV_AA);
			if(i<currentstep)  circle(show,  toimage(route[i].x, route[i].y), 3, CV_RGB(0,255,0), -1,CV_AA);
			else circle(show,  toimage(route[i].x, route[i].y), 3, CV_RGB(255,0,0), -1,CV_AA);
			}
		if(route.size()){
			circle(show,  toimage(route.back().x, route.back().y), 3, CV_RGB(255,0,0), -1,CV_AA);
			}
		}

	/*
	for(int i=0; i<h1.cols; i++){
		line(show, Point( 10+i, cvRound(160-5.0*h1.at<float>(i)) ) ,
                       Point( 10+i, 160 ),
                       Scalar( 255, 0, 0), 1, 8, 0  );
		}
	for(int i=0; i<h2.cols; i++){
		line(show, Point( 500+i, cvRound(160-5.0*h2.at<float>(i)) ) ,
                       Point( 500+i, 160 ),
                       Scalar( 255, 0, 255), 1, 8, 0  );
		}
	*/
	}


void loaddistancematrix(){
	tic();
	FileStorage fs;
	printf("Opening data file...\n");
	fs.open("distances.yml", FileStorage::READ);
	if(fs.isOpened()){
		//file already computed, load it
		printf("Loading data...\n");
		fs["distances"] >> distances;
		fs["freespace"] >> freespace;
		}
	else{
		//we need to compute it
		static float max_range=31.0/0.05;
		vector<Point> obstacles;
		Mat image;
		cvtColor(mapa,image,CV_BGR2GRAY);
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
		distances = cv::Mat::zeros(freespace.size(), 360, CV_32FC1)+max_range;
		for(unsigned int j=0; j<freespace.size(); j++){ //para cada espacio libre
			if(j%100==0) printf("Computed %d of %d\n",j,(int)freespace.size());
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
		}
	fs.release();
	cout << "Total time:" << toc()/1000 << endl;
	}



void manageRoutes(){
	static ros::NodeHandle n;
	static ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	if(recording){
		if(route.size()==0){
			route.push_back(poseActual);
			printf("Saved a new point to the route:(%f,%f,%f)\n",poseActual.x, poseActual.y, poseActual.z);
			}
		else if(norm(poseActual-route.back())>1.0 ){
			route.push_back(poseActual);
			printf("Saved a new point to the route:(%f,%f,%f)\n",poseActual.x, poseActual.y, poseActual.z);
			}
		
		}
	if(reproducing){
		if(currentstep<(int)route.size()){
			if(ordercompleted){
				currentstep++;
				geometry_msgs::PoseStamped msg;
				//we'll send a goal to the robot
				msg.header.frame_id = "/map";
				msg.header.stamp = ros::Time::now();
				msg.pose.position.x=route[currentstep].x;
				msg.pose.position.y=route[currentstep].y;
				msg.pose.orientation=tf::createQuaternionMsgFromYaw(route[currentstep].z);
				printf("Going to a new point whitin the route: (%f,%f,%f)\n",route[currentstep].x, route[currentstep].y, route[currentstep].z);
				goal_pub.publish(msg);
				}
			//if(ordercancelled)
			}
		else reproducing=false;
		}
	}


int main(int argc, char **argv){
	char buf[200];
	cout << "Cargamos el mapa de " << getcwd(buf,200)  << string("/map.yaml") << endl;
	parseYamlMap(string(getcwd(buf,200)), string("map.yaml"));
	centrocrop=Point(mapa.cols/2,mapa.rows/2);
	

	ros::init(argc, argv, "sendgoal");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/amcl_pose", 2, AmclCallback);
	ros::Subscriber sub2 = n.subscribe("/particlecloud", 2, AmclPartCallback);
	ros::Subscriber sub3 = n.subscribe<sensor_msgs::LaserScan> ("/base_scan", 2, ScanCallback);
	ros::Subscriber sub4 = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 2, PathCallback);
	
    loaddistancematrix();

	namedWindow("map", CV_WINDOW_NORMAL || CV_WINDOW_FREERATIO || CV_GUI_NORMAL);
	setMouseCallback("map", onMouse, NULL);
	


	float scale=1.0;
	Mat toshow;
	for(;;){
		ros::spinOnce();
		drawGUI();
		manageRoutes();
		int width=show.cols*scale;
		int height=show.rows*scale;
		int x=centrocrop.x-width/2;
		int y=centrocrop.y-height/2;
		if(x<0) x=0;
		if(y<0) y=0;
		if( (x+width)>show.cols ) x=show.cols-width;
		if( (y+height)>show.rows ) y=show.rows-height;
		
		
		Rect crop=Rect(x,y, width, height);
		resize(show(crop), toshow, Size(mapa.cols,mapa.rows));
		imshow("map", toshow);
		char key = (char) cvWaitKey(1);
		if(key=='r'){
			drawData.particles.clear();
			drawData.positions.clear();
			}
		if( key=='i'){
			recording=!recording;
			if(recording) route.clear();
			reproducing=false;
			if(recording) printf("Recording: YES\n"); else printf("Recording: NO\n"); 
			}
		if( key=='u'){
			recording=false;
			reproducing=!reproducing;
			if(reproducing) printf("Reproducing: YES\n"); else printf("Reproducing: NO\n");
			}
		if( key=='o'){ 
			drawConfig.showPath=!drawConfig.showPath; 
			if(drawConfig.showparticles) printf("Showing path: YES\n"); else printf("Showing path: NO\n"); 
			}
		if( key=='p'){ 
			drawConfig.showparticles=!drawConfig.showparticles; 
			if(drawConfig.showparticles) printf("Showing particles: YES\n"); else printf("Showing particles: NO\n"); 
			}
		if(key=='g'){
			if(system("rosservice call /global_localization")) printf("Global init called\n");
			}
		if(key=='c'){ 
			cancel=true;}
		if(key=='-'){
			scale=scale*2;
			if(scale>1) scale=1;
			}
		if(key=='+'){
			scale=scale/2;
			if(scale<0.1) scale=0.1;
			}
		if(key=='w' || key==30){
			centrocrop.y=max(0,centrocrop.y-10);
			}
		if(key=='a' || key==29){
			centrocrop.x=max(0,centrocrop.x-10);
			}
		if(key=='d' || key==28){
			centrocrop.x=min(toshow.cols,centrocrop.x+10);
			}
		if(key=='s' || key==31){
			centrocrop.y=min(toshow.rows,centrocrop.y+10);
			}		
	

			
		if( key == 'q' || key == 27 ) break;
		}

  return 0;
}

/*
rosbag record /tf /scan /RosAria/pose
*/


/*
void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	static tf::TransformListener listener;
	tf::StampedTransform transform;
 	try{
		ros::Time now = ros::Time::now();
      	listener.lookupTransform("/base_laser_link", "/map", now, transform);
    	}
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    	}
	drawData.laserScan.clear();
	int nscans=(msg->angle_max-msg->angle_min)/msg->angle_increment;
	for (int i = 0; i<nscans; i++) {
		if(msg->ranges[i]>msg->range_min && msg->ranges[i]<msg->range_max){
			float x=transform.getOrigin().x()+msg->ranges[i]*
							cos(tf::getYaw(transform.getRotation())+msg->angle_min+msg->angle_increment*i);
			float y=transform.getOrigin().y()+msg->ranges[i]*
							sin(tf::getYaw(transform.getRotation())+msg->angle_min+msg->angle_increment*i);
			//drawData.laserScan.push_back(toimage(y,x));
			}
		}
	}
*/
