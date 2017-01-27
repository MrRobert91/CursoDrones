/*
 *  Copyright (C) 1997-2012 JDE Developers Teameldercare.camRGB
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Authors : Jose María Cañas <jmplaza@gsyc.es>
			Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>

 */

/** \file openniServer.cpp
 * \brief openniServer component main file
 */

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/kinectleds.h>
#include <jderobot/camera.h>
#include <jderobot/pose3dmotors.h>
#include <jderobot/pointcloud.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <tr1/memory>
#include <list>
#include <sstream>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "myprogeo.h"
#include <OpenNI.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/video/background_segm.hpp>
#include <signal.h>
#include <log/Logger.h>
#include <ns/ns.h>
#include <zlib.h>
#include <openssl/md5.h>

#include "easyiceconfig/EasyIce.h" 

#ifdef WITH_NITE2
#include "NiTE.h"
#endif


#define VID_MICROSOFT 0x45e
#define PID_NUI_MOTOR 0x02b0
#define NUM_THREADS 5
#define MAX_LENGHT 10000
#define SAMPLE_READ_WAIT_TIMEOUT 2000
#define RETRY_MAX_TIMES 5

#define MAX_TIMES_PREHEATING 10
#define SLEEP_PREHEATING 10

#define CHECK_RC(rc, what)                                      \
		if (rc != openni::STATUS_OK)                                         \
		{                                                               \
			std::cerr << what << " failed: " << std::string(openni::OpenNI::getExtendedError()) << std::endl;     \
			\
		}


#ifdef WITH_NITE2
nite::UserTracker* m_pUserTracker;
nite::UserTrackerFrameRef userTrackerFrame;
nite::Status rcN;
#endif





//global configuration
openni::VideoFrameRef		m_depthFrame;
openni::VideoFrameRef		m_colorFrame;
openni::Device			m_device;
int cameraR, cameraD,cameraIR, ImageRegistration;
int colors[10][3];
int SELCAM;
IceUtil::Mutex mutex;
bool componentAlive;
pthread_t updateThread;
int deviceMode; //videmode for device streamings

int retry_times = 0;

//block to wait the device initialization
IceUtil::Mutex controlMutex;
IceUtil::Cond sem;
int mirrorDepth, mirrorRGB;


namespace openniServer{


std::vector<int> distances;
std::vector<int> pixelsID;
cv::Mat* srcRGB;
int userGeneratorActive=0;
openni::VideoStream depth, color, ir;
openni::VideoStream** m_streams;
openni::VideoMode depthVideoMode;
openni::VideoMode colorVideoMode;




int segmentationType; //0 ninguna, 1 NITE
int mainFPS;


void* updateThread(void*)
{

	openni::Status rc = openni::STATUS_OK;

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		jderobot::Logger::getInstance()->error(  boost::lexical_cast<std::string>(SELCAM) + ": Initialize failed: "+  std::string(openni::OpenNI::getExtendedError()));
	}
	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);

	const char* deviceUri;
	//checking the number off connected devices
	if (deviceList.getSize() < 1)
	{
		jderobot::Logger::getInstance()->error( "Missing devices" );
		openni::OpenNI::shutdown();
	}

	//getting the Uri of the selected device
	deviceUri = deviceList[SELCAM].getUri();



	//getting the device from the uri
	openni::VideoStream depth;
	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		jderobot::Logger::getInstance()->error( boost::lexical_cast<std::string>(SELCAM) + " : Couldn't open device " + std::string(deviceUri) + ": " + std::string(std::string(openni::OpenNI::getExtendedError())));
		openni::OpenNI::shutdown();
	}




	//depth
	if (cameraD){
		rc = depth.create(m_device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			//rc = depth.start();
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->error( "OpenniServer: Couldn't start depth stream: "+ std::string(std::string(openni::OpenNI::getExtendedError())) );
				depth.destroy();
			}
		}
		else
		{
			jderobot::Logger::getInstance()->error( "OpenniServer: Couldn't find depth stream: " +  std::string(openni::OpenNI::getExtendedError()) );
		}

		if (mirrorDepth){
			rc=depth.setMirroringEnabled(true);
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->error( "OpenniServer: error at set depth mirror: " + std::string(openni::OpenNI::getExtendedError()) );
			}
		}
		else{
			rc=depth.setMirroringEnabled(false);
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->error( "OpenniServer: error at set depth mirror: " + std::string(openni::OpenNI::getExtendedError()) );
			}
		}
	}

	//color
	if (cameraR){
		rc = color.create(m_device, openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			//rc = color.start();
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->error( "OpenniServer: Couldn't start color stream: " + std::string(openni::OpenNI::getExtendedError()) );
				color.destroy();
			}
		}
		else
		{
			jderobot::Logger::getInstance()->error( "OpenniServer: Couldn't find color stream: " + std::string(openni::OpenNI::getExtendedError()) );
		}
		if (mirrorRGB){
			rc=color.setMirroringEnabled(true);
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->error( "OpenniServer: error at set color mirror: " + std::string(openni::OpenNI::getExtendedError()) );
			}
		}
		else{
			rc=color.setMirroringEnabled(false);
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->error("OpenniServer: error at set color mirror: " + std::string(openni::OpenNI::getExtendedError()) );
			}
		}
	}




	if (cameraD){
		const openni::SensorInfo *depthSensorInfo = m_device.getSensorInfo(openni::SENSOR_DEPTH);
		rc= depth.setVideoMode(depthSensorInfo->getSupportedVideoModes()[deviceMode]);
		if (rc != openni::STATUS_OK)
		{
			jderobot::Logger::getInstance()->error( "OpenniServer: error at set depth videoMode: " + std::string(openni::OpenNI::getExtendedError()) );
		}
		jderobot::Logger::getInstance()->info( "OpenniServer: depth modes" );
		for(int i=0;i < depthSensorInfo->getSupportedVideoModes().getSize();i++)
		{
			openni::VideoMode videoMode = depthSensorInfo->getSupportedVideoModes()[i];
			jderobot::Logger::getInstance()->info( "fps: " + boost::lexical_cast<std::string>(videoMode.getFps()) + "x: " + boost::lexical_cast<std::string>(videoMode.getResolutionX()) + "y " +  boost::lexical_cast<std::string>(videoMode.getResolutionY()) );
		}
		depthVideoMode = depth.getVideoMode();
		depth.start();

	}

	if (cameraR){
		const openni::SensorInfo *colorSensorInfo = m_device.getSensorInfo(openni::SENSOR_COLOR);
		rc= color.setVideoMode(colorSensorInfo->getSupportedVideoModes()[deviceMode]);
		if (rc != openni::STATUS_OK)
		{
			jderobot::Logger::getInstance()->error("OpenniServer: error at set color videoMode: " + std::string(openni::OpenNI::getExtendedError()) );
			color.destroy();
		}
		jderobot::Logger::getInstance()->info( "OpenniServer color modes:" );
		for(int i=0;i < colorSensorInfo->getSupportedVideoModes().getSize();i++)
		{
			openni::VideoMode videoMode = colorSensorInfo->getSupportedVideoModes()[i];
			jderobot::Logger::getInstance()->info( "fps: " + boost::lexical_cast<std::string>(videoMode.getFps()) + "x: " + boost::lexical_cast<std::string>(videoMode.getResolutionX()) + "y " +  boost::lexical_cast<std::string>(videoMode.getResolutionY()) );
		}
		colorVideoMode = color.getVideoMode();
		srcRGB = new cv::Mat(cv::Size(colorVideoMode.getResolutionX(),colorVideoMode.getResolutionY()),CV_8UC3);
		color.start();
	}



	if ((cameraR)&&(cameraD)&&(ImageRegistration)){
		rc=m_device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

		if (rc != openni::STATUS_OK)
		{
			jderobot::Logger::getInstance()->error( "OpenniServer: error at set registration: " + std::string(openni::OpenNI::getExtendedError()) );
		}
	}




	jderobot::Logger::getInstance()->info( "COLOR: fps:" + boost::lexical_cast<std::string>(color.getVideoMode().getFps()) + "w" + boost::lexical_cast<std::string>(color.getVideoMode().getResolutionX()) + "h" + boost::lexical_cast<std::string>(color.getVideoMode().getResolutionY()) );
	jderobot::Logger::getInstance()->info( "DEPTH: fps:" + boost::lexical_cast<std::string>(depth.getVideoMode().getFps()) + "w" + boost::lexical_cast<std::string>(depth.getVideoMode().getResolutionX()) + "h" + boost::lexical_cast<std::string>(depth.getVideoMode().getResolutionY()));

	if (cameraR && cameraD){
		rc=m_device.setDepthColorSyncEnabled(true);
		if (rc != openni::STATUS_OK)
		{
			jderobot::Logger::getInstance()->error( "OpenniServer: error at set syncronization: " + std::string(openni::OpenNI::getExtendedError()) );
		}
		if (depth.isValid() && color.isValid())
		{

			int depthWidth = depthVideoMode.getResolutionX();
			int depthHeight = depthVideoMode.getResolutionY();
			int colorWidth = colorVideoMode.getResolutionX();
			int colorHeight = colorVideoMode.getResolutionY();

			if (depthWidth == colorWidth &&
					depthHeight == colorHeight)
			{
			}
			else
			{
				jderobot::Logger::getInstance()->error(  "Error - expect color and depth to be in same resolution: D: " + boost::lexical_cast<std::string>(depthWidth) + "x" + boost::lexical_cast<std::string>(depthHeight) + "C: " + boost::lexical_cast<std::string>(colorWidth) + "x" +  boost::lexical_cast<std::string>(colorHeight));


			}
		}
	}

	distances.resize(depth.getVideoMode().getResolutionX()*depth.getVideoMode().getResolutionY());
	pixelsID.resize(depth.getVideoMode().getResolutionX()*depth.getVideoMode().getResolutionY());



	//NITE
#ifdef WITH_NITE2

	if (segmentationType){
		m_pUserTracker = new nite::UserTracker;
		nite::NiTE::initialize();

		if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
		{
			jderobot::Logger::getInstance()->error( "OpenniServer: Couldn't create userTracker " );
		}
	}
#endif

	m_streams = new openni::VideoStream*[2];
	m_streams[0] = &depth;
	m_streams[1] = &color;

	jderobot::Logger::getInstance()->info("Starting device pre-heating .... ");
	// Repeat at least MAX_TIMES_PREHEATING to wait Xtion is not cold
	for (int i=0; i<MAX_TIMES_PREHEATING; i++)
	{
		int changedIndex;
		openni::Status rc;
		rc=openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex,SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != openni::STATUS_OK)
		{
			jderobot::Logger::getInstance()->warning( "Wait failed! (timeout is " + boost::lexical_cast<std::string>(SAMPLE_READ_WAIT_TIMEOUT) +  "ms) " + std::string(openni::OpenNI::getExtendedError()) );
			retry_times++;
			if (retry_times > RETRY_MAX_TIMES)
			{
				jderobot::Logger::getInstance()->error( "Retry Max Times exceeded!. Force Exit!!" );
				exit(-1);
			}
			continue;
		}


		if (changedIndex != 0)
			continue;

		openni::VideoFrameRef mAuxFrame;
		depth.readFrame( &mAuxFrame );
	}
	sleep(SLEEP_PREHEATING);
	jderobot::Logger::getInstance()->info("End device pre-heating");

	sem.broadcast();


	//diferente en arm que en x86???
	float cycle=(float)(1/(float)mainFPS)*1000000;
	IceUtil::Time lastIT=IceUtil::Time::now();
	bool first=true;




	while(componentAlive){


		int changedIndex;

		openni::Status rc;
		try{
			rc=openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex,SAMPLE_READ_WAIT_TIMEOUT);
			if (rc != openni::STATUS_OK)
			{
				jderobot::Logger::getInstance()->warning( "Wait failed! (timeout is " + boost::lexical_cast<std::string>(SAMPLE_READ_WAIT_TIMEOUT) +  "ms) " + std::string(openni::OpenNI::getExtendedError()) );
				retry_times++;
				if (retry_times > RETRY_MAX_TIMES)
				{
					jderobot::Logger::getInstance()->error( "Retry Max Times exceeded!. Force Exit!!" );
					exit(-1);
				}
				continue;
			}
		}
		catch ( std::exception& ex) {
			std::cerr << ex.what() << std::endl;
		}

		if (rc != openni::STATUS_OK)
		{
			jderobot::Logger::getInstance()->warning( "Wait failed" );
		}
		else if(first){
			jderobot::Logger::getInstance()->info( "OpenniServer initialized" );
			first=false;
		}

		mutex.lock();

		switch (changedIndex)
		{
		case 0:
			depth.readFrame(&m_depthFrame);
			break;
		case 1:
			color.readFrame(&m_colorFrame);
			break;
		}



		//nite

#ifdef WITH_NITE2
		if (segmentationType){
			rcN = m_pUserTracker->readFrame(&userTrackerFrame);
			m_depthFrame = userTrackerFrame.getDepthFrame();
			if (rcN != nite::STATUS_OK)
			{
				std::cout << "GetNextData failed" << std::endl;
				//return;
			}
		}
#endif



		mutex.unlock();



		int process = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();

		if (process > (int)cycle ){
			jderobot::Logger::getInstance()->warning("-------- openniServer: Main openni timeout-" );
		}
		else{
			int delay = (int)cycle - process;
			if (delay <1 || delay > (int)cycle)
				delay = 1;

			usleep(delay);
		}

		lastIT=IceUtil::Time::now();

	}
	if (cameraD){
		depth.stop();
		depth.destroy();
	}
	if (cameraR){
		color.stop();
		color.destroy();
	}

#ifdef WITH_NITE2
	if (segmentationType){
		nite::NiTE::shutdown();
	}
#endif


	return NULL;
}



/**
 * \brief Class which contains all the functions and variables to make run the Robot Cameras
 */
class CameraRGB: virtual public jderobot::Camera {
public:
	CameraRGB(std::string& propertyPrefix, const Ice::PropertiesPtr propIn)
: prefix(propertyPrefix),
  imageFmt(),
  imageDescription(new jderobot::ImageDescription()),
  cameraDescription(new jderobot::CameraDescription()),
  replyTask()
{
		Ice::PropertiesPtr prop = propIn;

		//fill cameraDescription
		cameraDescription->name = prop->getProperty(prefix+"Name");
		if (cameraDescription->name.size() == 0)
			jderobot::Logger::getInstance()->warning( "Camera name not configured" );

		cameraDescription->shortDescription = prop->getProperty(prefix + "ShortDescription");

		//fill imageDescription
		imageDescription->width = colorVideoMode.getResolutionX();
		imageDescription->height = colorVideoMode.getResolutionY();


		int playerdetection = prop->getPropertyAsIntWithDefault(prefix+"PlayerDetection",0);

#ifndef WITH_NITE2
		playerdetection=0;
#endif
		int fps = prop->getPropertyAsIntWithDefault(prefix+"fps",5);
		//we use formats according to colorspaces
		std::string fmtStr = prop->getPropertyWithDefault(prefix+"Format","YUY2");//default format YUY2
		imageFmt = colorspaces::Image::Format::searchFormat(fmtStr);
		if (!imageFmt)
			jderobot::Logger::getInstance()->warning( "Format " + fmtStr + " unknown" );
		imageDescription->size = imageDescription->width * imageDescription->height * CV_ELEM_SIZE(imageFmt->cvType);
		imageDescription->format = imageFmt->name;

		// Set the formats allowed
		mFormats.push_back(colorspaces::ImageRGB8::FORMAT_RGB8.get()->name);
		mFormats.push_back(colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name);

		jderobot::Logger::getInstance()->info( "Starting thread for camera: " + cameraDescription->name );
		replyTask = new ReplyTask(this,fps, playerdetection, mFormats[0]);

		this->control=replyTask->start();//my own thread
}

	virtual ~CameraRGB(){
		jderobot::Logger::getInstance()->info( "Stopping and joining thread for camera: " + cameraDescription->name );
		replyTask->destroy();
		this->control.join();
	}

	virtual jderobot::ImageDescriptionPtr getImageDescription(const Ice::Current& c){
		return imageDescription;
	}

	virtual jderobot::CameraDescriptionPtr getCameraDescription(const Ice::Current& c){
		return cameraDescription;
	}

	virtual jderobot::ImageFormat getImageFormat(const Ice::Current& c)
	{
		return mFormats;
	}

	virtual void getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr& cb,const std::string& format, const Ice::Current& c){
		replyTask->pushJob(cb, format);
	}

	virtual std::string startCameraStreaming(const Ice::Current&){
		jderobot::Logger::getInstance()->info( "Should be made anything to start camera streaming: " + cameraDescription->name);
		return std::string("");
	}

	virtual void stopCameraStreaming(const Ice::Current&) {
		jderobot::Logger::getInstance()->info( "Should be made anything to stop camera streaming: " +  cameraDescription->name );
	}
	virtual void reset(const Ice::Current&)
	{
	}

	virtual Ice::Int setCameraDescription(const jderobot::CameraDescriptionPtr&, const Ice::Current&){
		return 0;
	}

private:
	class ReplyTask: public IceUtil::Thread{
	public:
		ReplyTask(CameraRGB* camera, int fps, int playerdetection, std::string format ):mycameravga(camera),_done(false), mFormat(format) {
			segmentation=playerdetection;
			this->fps=fps;
		}


		void pushJob(const jderobot::AMD_ImageProvider_getImageDataPtr& cb, std::string format){
			mFormat = format;

			IceUtil::Mutex::Lock sync(requestsMutex);
			requests.push_back(cb);
		}

		void print_md5_sum(unsigned char* md) {
			int i;
			for(i=0; i <MD5_DIGEST_LENGTH; i++) {
				printf("%02x",md[i]);
			}
		}

		virtual void run(){

			jderobot::ImageDataPtr reply(new jderobot::ImageData);
			reply->description = mycameravga->imageDescription;
			reply->pixelData.resize(mycameravga->imageDescription->width*mycameravga->imageDescription->height*3);
			cv::Mat dst_resize;



			float cycle; // duración del ciclo

			cycle=(float)(1/(float)fps)*1000000;

			IceUtil::Time lastIT=IceUtil::Time::now();
			while(!(_done)){
				mutex.lock();
				IceUtil::Time t = IceUtil::Time::now();
				reply->timeStamp.seconds = (long)t.toSeconds();
				reply->timeStamp.useconds = (long)t.toMicroSeconds() - reply->timeStamp.seconds*1000000;

				if (!m_colorFrame.isValid()){
					mutex.unlock();
					continue;
				}
				//nite

#ifdef WITH_NITE2
				const nite::UserId* pLabels;
				if (segmentation){
					const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
					pLabels= userLabels.getPixels();
				}
#endif

				const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
				int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);



				for (int y = 0; y < m_colorFrame.getHeight(); ++y)
				{
					const openni::RGB888Pixel* pImage = pImageRow;
					for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage)
					{
						switch(segmentation){
						case 0:
							srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 0] = pImage->r;
							srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 1] = pImage->g;
							srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 2] = pImage->b;
							break;
						case 1:
#ifdef WITH_NITE2
							if (segmentation){
								pixelsID[(y*m_colorFrame.getWidth() + x)]= *pLabels;
								if (*pLabels!=0)
								{
									srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 0] = colors[*pLabels][0];
									srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 1] = colors[*pLabels][1];
									srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 2] = colors[*pLabels][2];
								}
								else{
									srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 0] = 0;
									srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 1] = 0;
									srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 2] = 0;
								}
								++pLabels;
							}
							else{
								srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 0] = pImage->r;
								srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 1] = pImage->g;
								srcRGB->data[(y*m_colorFrame.getWidth() + x)*3 + 2] = pImage->b;
							}
#endif
							break;
						case 2:

						default:
							jderobot::Logger::getInstance()->error( "openniServer: Error segmentation not supported" );
							break;
						}


					}
					pImageRow += rowSize;
				}

				if ((mycameravga->imageDescription->width != m_colorFrame.getWidth()) ||
						(mycameravga->imageDescription->height != m_colorFrame.getHeight())){

					jderobot::Logger::getInstance()->warning( "Assuming kinect device with resampled on device not working" );
					resize(*srcRGB, dst_resize, srcRGB->size(), 0, 0, cv::INTER_LINEAR);
					memcpy(&(reply->pixelData[0]),(unsigned char *) dst_resize.data,dst_resize.cols*dst_resize.rows * 3);
				}
				else{

					if (mFormat == colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name)
					{
						unsigned long source_len = srcRGB->rows*srcRGB->cols*3;
						unsigned long compress_len = compressBound(source_len);
						unsigned char* compress_buf = (unsigned char *) malloc(compress_len);

						int r = compress((Bytef *) compress_buf, (uLongf *) &compress_len, (const Bytef *) &(srcRGB->data[0]), (uLong)source_len );

						if(r != Z_OK) {
							jderobot::Logger::getInstance()->error("Compression Error");
							switch(r) {
							case Z_MEM_ERROR:
								jderobot::Logger::getInstance()->error("Compression Error: Not enough memory to compress");
								break;
							case Z_BUF_ERROR:
								jderobot::Logger::getInstance()->error("Compression Error: Target buffer too small.");
								break;
							case Z_STREAM_ERROR:
								jderobot::Logger::getInstance()->error("Compression Error: Invalid compression level.");
								break;
							}
						}
						else
						{
							reply->description->format = colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name;
							memcpy(&(reply->pixelData[0]),  &(compress_buf[0]), compress_len);

							// md5sum
							unsigned char *md5hash;

							md5hash = MD5((const unsigned char*) &(compress_buf[0]), compress_len, NULL);

							std::stringstream buf;
							for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
								buf << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(static_cast<unsigned char>(md5hash[i]));
							reply->description->md5sum = buf.str();

						}

						if (compress_buf)
							free(compress_buf);

					}
					else if (mFormat == colorspaces::ImageRGB8::FORMAT_RGB8.get()->name)
					{
						reply->description->format = colorspaces::ImageRGB8::FORMAT_RGB8.get()->name;
						memcpy(&(reply->pixelData[0]),(unsigned char *) srcRGB->data, srcRGB->rows*srcRGB->cols * 3);

						// md5sum
						unsigned char *md5hash;
						md5hash = MD5((const unsigned char*) (unsigned char *) srcRGB->data, srcRGB->rows*srcRGB->cols * 3, NULL);

						std::stringstream buf;
						for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
							buf << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(static_cast<unsigned char>(md5hash[i]));

						reply->description->md5sum = buf.str();

					}
					else
					{
						jderobot::Logger::getInstance()->error("Format image not recognized: " + mFormat);
					}

				}

				{//critical region start
					IceUtil::Mutex::Lock sync(requestsMutex);
					while(!requests.empty()){
						jderobot::AMD_ImageProvider_getImageDataPtr cb = requests.front();
						requests.pop_front();
						cb->ice_response(reply);
					}

				}//critical region end
				mutex.unlock();


				int process = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();

				if (process > (int)cycle ){
					jderobot::Logger::getInstance()->warning("-------- openniServer: RGB openni timeout-" );
				}
				else{
					int delay = (int)cycle - process;
					if (delay <1 || delay > (int)cycle)
						delay = 1;

					usleep(delay);
				}

				lastIT=IceUtil::Time::now();
			}
		}
		virtual void destroy(){
			this->_done=true;
		}


	private:
		CameraRGB* mycameravga;
		IceUtil::Mutex requestsMutex;
		std::list<jderobot::AMD_ImageProvider_getImageDataPtr> requests;
		int segmentation;
		int fps;
		bool _done;
		std::string mFormat;

	};
	typedef IceUtil::Handle<ReplyTask> ReplyTaskPtr;

	std::string prefix;

	colorspaces::Image::FormatPtr imageFmt;
	jderobot::ImageDescriptionPtr imageDescription;
	jderobot::CameraDescriptionPtr cameraDescription;
	ReplyTaskPtr replyTask;
	IceUtil::ThreadControl control;
	jderobot::ImageFormat mFormats;


};







//*********************************************************************/
class CameraDEPTH: virtual public jderobot::Camera {
public:
	CameraDEPTH(std::string& propertyPrefix, const Ice::PropertiesPtr propIn)
: prefix(propertyPrefix),
  imageFmt(),
  imageDescription(new jderobot::ImageDescription()),
  cameraDescription(new jderobot::CameraDescription()),
  replyTask()
{


		Ice::PropertiesPtr prop = propIn;

		//fill cameraDescription
		cameraDescription->name = prop->getProperty(prefix+"Name");
		if (cameraDescription->name.size() == 0)
			jderobot::Logger::getInstance()->warning( "Camera name not configured" );

		cameraDescription->shortDescription = prop->getProperty(prefix+"ShortDescription");

		//fill imageDescription
		imageDescription->width = depthVideoMode.getResolutionX();
		int playerdetection = prop->getPropertyAsIntWithDefault(prefix+"PlayerDetection",0);
#ifndef WITH_NITE2
		playerdetection=0;
#endif

		imageDescription->height = depthVideoMode.getResolutionY();
		int fps = prop->getPropertyAsIntWithDefault(prefix+"fps",5);
		//we use formats acording to colorspaces
		std::string fmtStr = prop->getPropertyWithDefault(prefix+"Format","YUY2");//default format YUY2
		imageFmt = colorspaces::Image::Format::searchFormat(fmtStr);
		if (!imageFmt)
			jderobot::Logger::getInstance()->info( "Format " +  fmtStr + " unknown" );
		imageDescription->size = imageDescription->width * imageDescription->height * CV_ELEM_SIZE(imageFmt->cvType);
		imageDescription->format = imageFmt->name;

		// Image formats allowed
		mFormats.push_back(colorspaces::ImageRGB8::FORMAT_DEPTH8_16.get()->name);
		mFormats.push_back(colorspaces::ImageRGB8::FORMAT_DEPTH8_16_Z.get()->name);

		jderobot::Logger::getInstance()->info( "Starting thread for camera: " +  cameraDescription->name );
		replyTask = new ReplyTask(this, imageDescription->width, imageDescription->height,fps, playerdetection, mFormats[0]);


		this->control=replyTask->start();//my own thread
}

	virtual ~CameraDEPTH(){
		jderobot::Logger::getInstance()->info( "Stopping and joining thread for camera: " + cameraDescription->name );

		replyTask->destroy();
		this->control.join();
	}

	virtual jderobot::ImageDescriptionPtr getImageDescription(const Ice::Current& c){
		return imageDescription;
	}

	virtual jderobot::CameraDescriptionPtr getCameraDescription(const Ice::Current& c){
		return cameraDescription;
	}

	virtual void getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr& cb,const std::string& format, const Ice::Current& c){
		replyTask->pushJob(cb, format);
	}

	virtual jderobot::ImageFormat getImageFormat(const Ice::Current& c)
	{
		return mFormats;

	}

	virtual std::string startCameraStreaming(const Ice::Current&){
		jderobot::Logger::getInstance()->info( "Should be made anything to start camera streaming: " + cameraDescription->name );
		return std::string("");
	}

	virtual void stopCameraStreaming(const Ice::Current&) {
		jderobot::Logger::getInstance()->info( "Should be made anything to stop camera streaming: "  + cameraDescription->name );
	}
	virtual void reset(const Ice::Current&)
	{
	}

	virtual Ice::Int setCameraDescription(const jderobot::CameraDescriptionPtr&, const Ice::Current&){
		return 0;
	}

private:
	class ReplyTask: public IceUtil::Thread{
	public:
		ReplyTask(CameraDEPTH* camera, int width, int height, int fps, int playerDetection, std::string format)
		:mycameradepth(camera),_done(false), mFormat(format) {
			segmentation=playerDetection;
			this->fps=fps;
			this->minToTrain=15;
		}


		void pushJob(const jderobot::AMD_ImageProvider_getImageDataPtr& cb, std::string format){
			mFormat = format;
			IceUtil::Mutex::Lock sync(requestsMutex);
			requests.push_back(cb);
		}

		virtual void run(){
			int test;


			jderobot::ImageDataPtr reply(new jderobot::ImageData);
			reply->description = mycameradepth->imageDescription;
			reply->pixelData.resize(mycameradepth->imageDescription->width*mycameradepth->imageDescription->height*3);
			cv::Mat dst_resize(cv::Size(mycameradepth->imageDescription->width, mycameradepth->imageDescription->height),CV_8UC3);
			cv::Mat src(cv::Size(mycameradepth->imageDescription->width, mycameradepth->imageDescription->height),CV_8UC3);
			int cycle; // duración del ciclo
			IceUtil::Time lastIT;


			cycle=(float)(1/(float)fps)*1000000;

			lastIT=IceUtil::Time::now();
			while(!(_done)){
				mutex.lock();
				src=cv::Scalar(0, 0, 0);

				IceUtil::Time t = IceUtil::Time::now();
				reply->timeStamp.seconds = (long)t.toSeconds();
				reply->timeStamp.useconds = (long)t.toMicroSeconds() - reply->timeStamp.seconds*1000000;
				if (!m_depthFrame.isValid()){
					mutex.unlock();
					continue;
				}

				//cvZero(src);
				//nite
#ifdef WITH_NITE2
				const nite::UserId* pLabels;
				if (segmentation){
					const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
					pLabels= userLabels.getPixels();
				}
#endif



				const openni::DepthPixel* pDepth = (const openni::DepthPixel*)m_depthFrame.getData();
				int restOfRow = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel) - m_depthFrame.getWidth();

				for (int y = 0; y < m_depthFrame.getHeight(); ++y)
				{
					for (int x = 0; x < m_depthFrame.getWidth(); ++x, ++pDepth)
					{
						switch(segmentation){
						case 0:
							distances[(y*m_depthFrame.getWidth() + x)] = *pDepth;
							if (*pDepth != 0)
							{
								src.data[(y*m_depthFrame.getWidth()+ x)*3+0] = (float(*pDepth)/(float)MAX_LENGHT)*255.;
								src.data[(y*m_depthFrame.getWidth()+ x)*3+1] = (*pDepth)>>8;
								src.data[(y*m_depthFrame.getWidth()+ x)*3+2] = (*pDepth)&0xff;
							}
							break;
						case 1:
#ifdef WITH_NITE2
							if ((*pLabels!=0)||(!segmentation)){
								distances[(y*m_depthFrame.getWidth() + x)] = *pDepth;
								if (*pDepth != 0)
								{
									src.data[(y*m_depthFrame.getWidth()+ x)*3+0] = (float(*pDepth)/(float)MAX_LENGHT)*255.;
									src.data[(y*m_depthFrame.getWidth()+ x)*3+1] = (*pDepth)>>8;
									src.data[(y*m_depthFrame.getWidth()+ x)*3+2] = (*pDepth)&0xff;

								}
								else{
									src.data[(y*m_depthFrame.getWidth()+ x)*3+0] = 0;
									src.data[(y*m_depthFrame.getWidth()+ x)*3+1] = 0;
									src.data[(y*m_depthFrame.getWidth()+ x)*3+2] = 0;
								}
							}
							else{
								src.data[(y*m_depthFrame.getWidth()+ x)*3+0] = 0;
								src.data[(y*m_depthFrame.getWidth()+ x)*3+1] = 0;
								src.data[(y*m_depthFrame.getWidth()+ x)*3+2] = 0;
							}
							++pLabels;
#endif
							break;
						case 2:
							break;
						default:
							jderobot::Logger::getInstance()->error( "openniServer: Error segmentation not supported" );
							break;
						}
					}

					pDepth += restOfRow;
				}
				/*if (debug==2){
				cv::imshow("OpenniServer DEPTH", src);
				cv::waitKey(1);
			}*/

				if ((mycameradepth->imageDescription->width != m_depthFrame.getWidth()) ||
						(mycameradepth->imageDescription->height != m_depthFrame.getHeight())){
					//cv::resize(src,dst_resize);
					cv::resize(src, dst_resize, dst_resize.size(), 0, 0, cv::INTER_LINEAR);
					jderobot::Logger::getInstance()->warning("Assuming kinect device with resampled on device not working" );
					memcpy(&(reply->pixelData[0]),(unsigned char *) dst_resize.data,dst_resize.cols*dst_resize.rows * 3);
				}
				else{

					if (mFormat == colorspaces::ImageRGB8::FORMAT_DEPTH8_16_Z.get()->name)
					{
						size_t source_len = src.rows*src.cols * 3;
						size_t compress_len = compressBound(source_len);
						unsigned char* compress_buf = (unsigned char *) malloc(compress_len);

						int r = compress2((Bytef *) compress_buf, (uLongf *) &compress_len, (Bytef *) &(src.data[0]), (uLong)source_len , 9);
						if(r != Z_OK) {
							jderobot::Logger::getInstance()->error("Compression Error");
							switch(r) {
							case Z_MEM_ERROR:
								jderobot::Logger::getInstance()->error("Compression Error: Not enough memory to compress");
								break;
							case Z_BUF_ERROR:
								jderobot::Logger::getInstance()->error("Compression Error: Target buffer too small");
								break;
							case Z_STREAM_ERROR:    // Invalid compression level
								jderobot::Logger::getInstance()->error("Compression Error: Invalid compression level");
								break;
							}
						}

						reply->description->format=colorspaces::ImageRGB8::FORMAT_DEPTH8_16_Z.get()->name;
						reply->pixelData.resize(compress_len);
						memcpy(&(reply->pixelData[0]), (unsigned char *) compress_buf, compress_len);

						if (compress_buf)
							free(compress_buf);
					}
					else if (mFormat == colorspaces::ImageRGB8::FORMAT_DEPTH8_16.get()->name)
					{
						memcpy(&(reply->pixelData[0]),(unsigned char *) src.data,src.cols*src.rows * 3);
					}
					else
					{
						// TODO: Raise exception
					}
				}

				{//critical region start
					IceUtil::Mutex::Lock sync(requestsMutex);
					while(!requests.empty()){
						jderobot::AMD_ImageProvider_getImageDataPtr cb = requests.front();
						requests.pop_front();
						cb->ice_response(reply);
					}
				}//critical region end
				mutex.unlock();


				int process = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();

				if (process > (int)cycle ){
					jderobot::Logger::getInstance()->warning("-------- openniServer: Depth openni timeout-" );
				}
				else{
					int delay = (int)cycle - process;
					if (delay <1 || delay > (int)cycle)
						delay = 1;

					usleep(delay);
				}

				lastIT=IceUtil::Time::now();
			}
		}

		virtual void destroy(){
			this->_done=true;
		}


	private:
		CameraDEPTH* mycameradepth;
		IceUtil::Mutex requestsMutex;
		std::list<jderobot::AMD_ImageProvider_getImageDataPtr> requests;
		std::string mFormat;

		int segmentation;
		int fps;
		int minToTrain;
		//cv::BackgroundSubtractorMOG2 bg;
		cv::Mat fore;
		cv::Mat trainImage;
		bool _done;


	};
	typedef IceUtil::Handle<ReplyTask> ReplyTaskPtr;


	std::string prefix;
	colorspaces::Image::FormatPtr imageFmt;
	jderobot::ImageDescriptionPtr imageDescription;
	jderobot::CameraDescriptionPtr cameraDescription;
	ReplyTaskPtr replyTask;
	IceUtil::ThreadControl control;
	jderobot::ImageFormat mFormats;
};

/**
 * \brief Class wich contains all the functions and variables to serve point cloud interface
 */

class pointCloudI: virtual public jderobot::pointCloud{
public:
	pointCloudI (std::string& propertyPrefix, const Ice::PropertiesPtr propIn):
		prefix(propertyPrefix),data(new jderobot::pointCloudData()) {
		Ice::PropertiesPtr prop = propIn;

		int playerdetection = prop->getPropertyAsIntWithDefault("openniServer.PlayerDetection",0);
		int fps =prop->getPropertyAsIntWithDefault("openniServer.pointCloud.Fps",10);
		bool extra =(bool)prop->getPropertyAsIntWithDefault("openniServer.ExtraCalibration",0);
#ifndef WITH_NITE2
		playerdetection=0;
#endif
		pthread_mutex_init(&this->localMutex, NULL);
		replyCloud = new ReplyCloud(this,prop->getProperty("openniServer.calibration"), playerdetection, depthVideoMode.getResolutionX(), depthVideoMode.getResolutionY(),fps, extra);
		this->control=replyCloud->start();
	}

	virtual ~pointCloudI(){
		jderobot::Logger::getInstance()->info( "Stopping and joining thread for pointCloud" );
		replyCloud->destroy();
		this->control.join();
	}


	virtual jderobot::pointCloudDataPtr getCloudData(const Ice::Current&){
		data=replyCloud->getCloud();
		return data;
	};

private:
	class ReplyCloud :public IceUtil::Thread{
	public:
		ReplyCloud (pointCloudI* pcloud, std::string filepath,  int playerDetection, int widthIn, int heightIn, int fpsIn, bool extra) : data(new jderobot::pointCloudData()), data2(new jderobot::pointCloudData()), _done(false)
	{
			path=filepath;
			segmentation=playerDetection;
			cWidth = widthIn;
			cHeight = heightIn;
			fps=fpsIn;
			myCloud=pcloud;
			mypro=NULL;
			withExtraCalibration=extra;
	}

		void run()
		{
			mypro= new openniServer::myprogeo(1,cWidth,cHeight);
			mypro->load_cam((char*)path.c_str(),0, cWidth, cHeight,withExtraCalibration );




			int cycle; // duración del ciclo


			cycle=(float)(1/(float)fps)*1000000;
			IceUtil::Time lastIT=IceUtil::Time::now();
			while(!(_done)){
				float distance;
				mutex.lock();
				//creamos una copia local de la imagen de color y de las distancias.
				cv::Mat localRGB;
				if (srcRGB->rows != 0)
					srcRGB->copyTo(localRGB);
				std::vector<int> localDistance(distances);
				mutex.unlock();
				pthread_mutex_lock(&(this->myCloud->localMutex));
				data2->p.clear();
				for( unsigned int i = 0 ; (i < cWidth*cHeight)&&(distances.size()>0); i=i+9) {
					distance=(float)localDistance[i];
					if (distance!=0){
						//if (((unsigned char)srcRGB->data[3*i]!=0) && ((unsigned char)srcRGB->data[3*i+1]!=0) && ((unsigned char)srcRGB->data[3*i+2]!=0)){
						float xp,yp,zp,camx,camy,camz;
						float ux,uy,uz;
						float x,y;
						float k;
						float c1x, c1y, c1z;
						float fx,fy,fz;
						float fmod;
						float t;
						float Fx,Fy,Fz;

						mypro->mybackproject(i % cWidth, i / cWidth, &xp, &yp, &zp, &camx, &camy, &camz,0);

						//vector unitario
						float modulo;

						modulo = sqrt(1/(((camx-xp)*(camx-xp))+((camy-yp)*(camy-yp))+((camz-zp)*(camz-zp))));
						mypro->mygetcamerafoa(&c1x, &c1y, &c1z, 0);

						fmod = sqrt(1/(((camx-c1x)*(camx-c1x))+((camy-c1y)*(camy-c1y))+((camz-c1z)*(camz-c1z))));
						fx = (c1x - camx)*fmod;
						fy = (c1y - camy)*fmod;
						fz = (c1z - camz) * fmod;
						ux = (xp-camx)*modulo;
						uy = (yp-camy)*modulo;
						uz = (zp-camz)*modulo;
						Fx= distance*fx + camx;
						Fy= distance*fy + camy;
						Fz= distance*fz + camz;
						// calculamos el punto real
						t = (-(fx*camx) + (fx*Fx) - (fy*camy) + (fy*Fy) - (fz*camz) + (fz*Fz))/((fx*ux) + (fy*uy) + (fz*uz));
						auxP.x=t*ux + camx;
						auxP.y=t*uy+ camy;
						auxP.z=t*uz + camz;


						if (withExtraCalibration){
							mypro->applyExtraCalibration(&auxP.x, &auxP.y, &auxP.z);
						}

						if ( segmentation){
							auxP.id=pixelsID[i];
						}
						if (srcRGB->rows != 0){
							auxP.r=(float)(int) (unsigned char)localRGB.data[3*i];
							auxP.g=(float)(int) (unsigned char)localRGB.data[3*i+1];
							auxP.b=(float)(int) (unsigned char)localRGB.data[3*i+2];
						}
						data2->p.push_back(auxP);
					}
					//}
				}
				pthread_mutex_unlock(&(this->myCloud->localMutex));

				int delay = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();
				if (delay > cycle ){
					jderobot::Logger::getInstance()->info("-------- openniServer: POINTCLOUD openni timeout-" );
				}
				else{
					if (delay <1 || delay > cycle)
						delay = 1;
					usleep(delay);
				}


				lastIT=IceUtil::Time::now();
			}
		}


		jderobot::pointCloudDataPtr getCloud()
		{
			pthread_mutex_lock(&(this->myCloud->localMutex));
			data->p=data2->p;
			pthread_mutex_unlock(&(this->myCloud->localMutex));
			return data;
		}

		virtual void destroy(){
			this->_done=true;
		}



	private:
		myprogeo *mypro;
		int cWidth;
		int cHeight;
		int fps;
		jderobot::pointCloudDataPtr data, data2;
		jderobot::RGBPoint auxP;
		std::string path;
		int segmentation;
		pointCloudI* myCloud;
		bool _done;
		bool withExtraCalibration;

	};

	typedef IceUtil::Handle<ReplyCloud> ReplyCloudPtr;
	ReplyCloudPtr replyCloud;
	std::string prefix;
	jderobot::pointCloudDataPtr data;
	pthread_mutex_t localMutex;
	IceUtil::ThreadControl control;



};
} //namespace


Ice::CommunicatorPtr ic;
bool killed;
openniServer::CameraRGB *camRGB;
openniServer::CameraDEPTH *camDEPTH;
openniServer::pointCloudI *pc1;
jderobot::ns* namingService = NULL;

void exitApplication(int s){


	killed=true;
	componentAlive=false;

	if (camRGB!= NULL)
		delete camRGB;
	if (camDEPTH != NULL)
		delete camDEPTH;
	if (pc1 != NULL){
		delete pc1;
	}

	// NamingService
	if (namingService != NULL)
	{
		namingService->unbindAll();

		delete(namingService);
	}

	ic->shutdown();



	pthread_join(updateThread, NULL);



	m_device.close();
	openni::OpenNI::shutdown();
	exit(0);

}


int main(int argc, char** argv){

	componentAlive=true;
	killed=false;
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = exitApplication;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);



	Ice::PropertiesPtr prop;


	try{
		ic = EasyIce::initialize(argc,argv);
		prop = ic->getProperties();
	}
	catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		return 1;
	}
	catch (const char* msg) {
		std::cerr <<"Error :" << msg << std::endl;
		return 1;
	}
	std::string componentPrefix("openniServer");


	// Analyze LOG section

	std::string logFile = prop->getProperty(componentPrefix + ".Log.File.Name");
	if (logFile.size()==0)
		jderobot::Logger::getInstance()->warning("You didn't set log file!");
	else
		jderobot::Logger::getInstance()->setFileLog(logFile);

	std::string logLevel = prop->getProperty(componentPrefix + ".Log.File.Level");
	if (logLevel.size()==0)
		jderobot::Logger::getInstance()->warning("You didn't set *.Log.File.Level key!");
	else
		jderobot::Logger::getInstance()->setFileLevel(jderobot::Levels(boost::lexical_cast<int>(logLevel)));

	std::string screenLevel = prop->getProperty(componentPrefix + ".Log.Screen.Level");
	if (screenLevel.size()==0)
		jderobot::Logger::getInstance()->warning("You didn't set *.Log.Screen.Level key!");
	else
		jderobot::Logger::getInstance()->setScreenLevel(jderobot::Levels(boost::lexical_cast<int>(screenLevel)));

	jderobot::Logger::getInstance()->info("Logger:: screenLevel=" + screenLevel + " logLevel=" + logLevel + " LogFile=" + logFile);




	cameraR = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraRGB",0);
	cameraD = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraDEPTH",0);
	ImageRegistration = prop->getPropertyAsIntWithDefault(componentPrefix + ".ImageRegistration",1);
	int motors = prop->getPropertyAsIntWithDefault(componentPrefix + ".Pose3DMotorsActive",0);
	int leds = prop->getPropertyAsIntWithDefault(componentPrefix + ".KinectLedsActive",0);
	int pointCloud = prop->getPropertyAsIntWithDefault(componentPrefix + ".pointCloudActive",0);
	openniServer::segmentationType= prop->getPropertyAsIntWithDefault(componentPrefix + ".PlayerDetection",0);
	mirrorDepth = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraDEPTH.Mirror",0);
	mirrorRGB = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraRGB.Mirror",0);
	deviceMode=prop->getPropertyAsIntWithDefault(componentPrefix + ".Mode", 0);
	openniServer::mainFPS=prop->getPropertyAsIntWithDefault(componentPrefix + ".Hz", 20);
	std::string Endpoints = prop->getProperty(componentPrefix + ".Endpoints");
	Ice::ObjectAdapterPtr adapter =ic->createObjectAdapterWithEndpoints(componentPrefix, Endpoints);

	if (openniServer::segmentationType){
		cameraR=1;
		cameraD=1;
	}

	SELCAM = prop->getPropertyAsIntWithDefault(componentPrefix + ".deviceId",0);
	jderobot::Logger::getInstance()->info( "OpenniServer: Selected device: " + SELCAM );
	int nCameras=0;


	/*COLORS*/
	colors[0][0]=0;
	colors[0][1]=0;
	colors[0][2]=255;
	colors[1][0]=0;
	colors[1][1]=255;
	colors[1][2]=255;
	colors[2][0]=255;
	colors[2][1]=255;
	colors[2][2]=0;
	colors[3][0]=255;
	colors[3][1]=0;
	colors[3][2]=0;
	colors[4][0]=0;
	colors[4][1]=255;
	colors[4][2]=0;
	colors[5][0]=255;
	colors[5][1]=255;
	colors[5][2]=0;
	colors[6][0]=0;
	colors[6][1]=0;
	colors[6][2]=0;
	colors[7][0]=150;
	colors[7][1]=150;
	colors[7][2]=0;
	colors[8][0]=150;
	colors[8][1]=150;
	colors[8][2]=150;
	colors[9][0]=0;
	colors[9][1]=150;
	colors[9][2]=150;

	nCameras=cameraR + cameraD;
	//g_context =  new xn::Context;
	if ((nCameras>0)||(pointCloud)){
		pthread_create(&updateThread, NULL, &openniServer::updateThread, NULL);
	}


	//bloqueo hasta que se inicialice el dispositivo
	IceUtil::Mutex::Lock sync(controlMutex);
	sem.wait(sync);

	sync.release();

	// Naming Service
	int nsActive = prop->getPropertyAsIntWithDefault("NamingService.Enabled", 0);

	if (nsActive)
	{
		std::string ns_proxy = prop->getProperty("NamingService.Proxy");
		try
		{
			namingService = new jderobot::ns(ic, ns_proxy);
		}
		catch (Ice::ConnectionRefusedException& ex)
		{
			jderobot::Logger::getInstance()->error("Impossible to connect with NameService!");
			exit(-1);
		}
	}

	if (cameraR){
		std::string objPrefix(componentPrefix + ".CameraRGB.");
		std::string cameraName = prop->getProperty(objPrefix + "Name");
		if (cameraName.size() == 0){//no name specified, we create one using the index
			cameraName = "cameraR";
			prop->setProperty(objPrefix + "Name",cameraName);//set the value
		}
		jderobot::Logger::getInstance()->info("Creating camera " + cameraName );
		camRGB = new openniServer::CameraRGB(objPrefix,prop);
		adapter->add(camRGB, ic->stringToIdentity(cameraName));
		jderobot::Logger::getInstance()->info("              -------- openniServer: Component: CameraRGB created successfully(" + Endpoints + "@" + cameraName );


		if (namingService)
			namingService->bind(cameraName, Endpoints, camRGB->ice_staticId());



	}

	if (cameraD){
		std::string objPrefix(componentPrefix + ".CameraDEPTH.");
		std::string cameraName = prop->getProperty(objPrefix + "Name");
		if (cameraName.size() == 0){//no name specified, we create one using the index
			cameraName = "cameraD";
			prop->setProperty(objPrefix + "Name",cameraName);//set the value
		}
		jderobot::Logger::getInstance()->info( "Creating camera " +  cameraName );
		camDEPTH = new openniServer::CameraDEPTH(objPrefix,prop);
		adapter->add(camDEPTH, ic->stringToIdentity(cameraName));
		//test camera ok
		jderobot::Logger::getInstance()->info("              -------- openniServer: Component: CameraDEPTH created successfully(" + Endpoints + "@" + cameraName );

		if (namingService)
			namingService->bind(cameraName, Endpoints, camDEPTH->ice_staticId());

	}

	if (pointCloud){
		std::string objPrefix(componentPrefix + ".PointCloud.");
		std::string Name = prop->getProperty(objPrefix + "Name");
		jderobot::Logger::getInstance()->info( "Creating pointcloud1 " + Name );
		pc1 = new openniServer::pointCloudI(objPrefix,prop);
		adapter->add(pc1 , ic->stringToIdentity(Name));
		jderobot::Logger::getInstance()->info("              -------- openniServer: Component: PointCloud created successfully(" + Endpoints + "@" + Name );
	}
	adapter->activate();
	ic->waitForShutdown();
	adapter->destroy();

	if (!killed)
		exitApplication(0);
	return 0;

}
