
//Libraries for DDS
#include "HelloWorldPublisher.h"
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/eClock.h>
#include <unistd.h>
#include <cstdio>

//Navio libraries
#include </home/pi/Navio2/C++/Navio/Common/Util.h>
#include <memory>

//Opencv
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//PID
#include <stdio.h>
#include "pid.h"
#define SETPOINT 320

#define READ_FAILED -1
using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

using namespace cv;
using namespace std;

double inc =0;
 //PID pid = PID(0.3, 1480, 1544, 0.8, 0.2, 0.6);
// PID pid = PID(0.3, 1544, 1455, 0.7, 0.2, 0.8);
PID pid = PID(0.1, 1544, 1475, 0.4, 0.2, 0.6);
             // Laranxa, Verde
HelloWorldPublisher::HelloWorldPublisher():mp_participant(nullptr),
mp_publisher(nullptr)
{


}

bool HelloWorldPublisher::init()
{
    m_Hello.forward(1099);
    m_Hello.direction(1523);
    m_Hello.emergencyStop(964);
    ParticipantAttributes PParam;
    PParam.rtps.defaultSendPort = 11511;
    PParam.rtps.use_IP6_to_send = true;
    PParam.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
    PParam.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = true;
    PParam.rtps.builtin.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
    PParam.rtps.builtin.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("Participant_pub");
    mp_participant = Domain::createParticipant(PParam);

    if(mp_participant==nullptr)
        return false;
    //REGISTER THE TYPE

    Domain::registerType(mp_participant,&m_type);

    //CREATE THE PUBLISHER
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = "HelloWorld";
    Wparam.topic.topicName = "HelloWorldTopic";
    Wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Wparam.topic.historyQos.depth = 30;
    Wparam.topic.resourceLimitsQos.max_samples = 50;
    Wparam.topic.resourceLimitsQos.allocated_samples = 20;
    Wparam.times.heartbeatPeriod.seconds = 2;
    Wparam.times.heartbeatPeriod.fraction = 200*1000*1000;
    Wparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    mp_publisher = Domain::createPublisher(mp_participant,Wparam,(PublisherListener*)&m_listener);
    if(mp_publisher == nullptr)
        return false;

    return true;

}

HelloWorldPublisher::~HelloWorldPublisher()
{
    // TODO Auto-generated destructor stub
    Domain::removeParticipant(mp_participant);
}



void HelloWorldPublisher::PubListener::onPublicationMatched(Publisher* /*pub*/,MatchingInfo& info)
{
    if(info.status == MATCHED_MATCHING)
    {
        std::cout << "Publisher matched"<<std::endl;
    }
    else
    {
        std::cout << "Publisher unmatched"<<std::endl;
    }
}


int HelloWorldPublisher::run(){

        if (check_apm()) {
        return 1;
    }


  VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }


 int iLastXg = -1; 
 int iLastYg = -1;
int iLastX = -1; 
 int iLastY = -1;

 //Capture a temporary image from the camera
 Mat imgTmp;
 cap.read(imgTmp); 

 //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
        for(int i = 0; i < 1500; i++){
                        m_Hello.direction(1502);
                        m_Hello.forward(1104);
                        mp_publisher->write((void*)&m_Hello);

                usleep(100);
        }
	for(int i = 0; i < 1500; i++){
                        m_Hello.direction(1502);
                        m_Hello.forward(1514);
                        mp_publisher->write((void*)&m_Hello);

                usleep(100);
        }


    while (true)
    {

	cout << "Ola mundo OPENCV" << endl;

	Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        if (!bSuccess) //if not success, break loop
       	{
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

 	Mat imgHSV;

 	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  	Mat imgGreen;
	//inRange(imgHSV, Scalar(36, 0, 0), Scalar(86, 255, 255), imgGreen); 
  	inRange(imgHSV, Scalar(38, 100, 100), Scalar(90, 255, 255), imgGreen); //Threshold the image

  	//morphological opening (removes small objects from the foreground)
  	erode(imgGreen, imgGreen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  	dilate( imgGreen, imgGreen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  	//morphological closing (removes small holes from the foreground)
  	dilate( imgGreen, imgGreen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  	erode(imgGreen, imgGreen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  	//Calculate the moments of the thresholded image
 	 Moments oMomentsg = moments(imgGreen);

  //	double dM01g = oMomentsg.m01;
  	double dM10g = oMomentsg.m10;
  	double dAreag = oMomentsg.m00;
	int forward = 1198;
	int direction = 1519;
	//int direction = 1480;
	int posXg = 0;
	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  	if (dAreag > 10000)
  	{
   		//calculate the position of the ball
   		posXg = dM10g / dAreag;
   //		int posYg = dM01g / dAreag;

   		cout<< "Eje X objeto verde:"<< posXg << "  ";
   /*		cout<< "Eje Y objeto verde:" <<  posYg << "\n";
   		if (iLastXg >= 0 && iLastYg >= 0 && posXg >= 0 && posYg >= 0)
   		{
    			//Draw a red line from the previous point to the current point
    			line(imgLines, Point(posXg, posYg), Point(iLastXg, iLastYg), Scalar(0,0,255), 2);
   		}

   		iLastXg = posXg;
   		iLastYg = posYg;*/
  	}

Mat imgOrange;

  inRange(imgHSV, Scalar(5, 111, 150), Scalar(15, 225, 255), imgOrange); //Threshold the image

  //morphological opening (removes small objects from the foreground)
  erode(imgOrange, imgOrange, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgOrange, imgOrange, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //morphological closing (removes small holes from the foreground)
  dilate( imgOrange, imgOrange, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgOrange, imgOrange, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgOrange);

 // double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;
	int posX = 640;
  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000)
  {
   //calculate the position of the ball
   posX = dM10 / dArea;
  // int posY = dM01 / dArea;        
   cout<< "Eje X objeto naranja:"<< posX << "  ";
  /* cout<< "Eje Y objeto naranja:" <<  posY << "\n";
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
   }

   iLastX = posX;
   iLastY = posY;*/
  }

  //              int posMedia = 320;
//int posMediaConos = (posX - posXg) / 2;


                /*if(posXg != (640 - posX)) {
			if(posXg < (640-posX))
                        {
			// LARANXA DERECHA
				direction = 1460;
				//direction = 1490;
			} else {
			// VERDE ESQUERDA
				direction = 1556;
				//direction = 1526;
			}
		}*/
/*		if(posMediaCOnos > posMedia) {
			direction = ;
		}
*/
		//calcular punto medio
		double pv= ((posXg-posX)/2)+posX;
		//aplicar PID
		inc = pid.calculate(SETPOINT, pv);
		printf("pv: %7.3f inc: %7.3f \n", pv, inc);
		direction = inc;
		
                        m_Hello.direction(direction);
                        m_Hello.forward(forward);
                        mp_publisher->write((void*)&m_Hello);
//                usleep(100);

}

}
