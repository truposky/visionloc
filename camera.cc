#include "camera.hh"
#include <dmtx.h>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pthread.h>
#include <vector>
#include <iostream>
#include <stdexcept>

namespace {
const char* about = "Detect ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        ;
}



Camera::Camera(int id_cam, int width, int height, double wc_height, double wc_offset_x,
double wc_offset_y, double wc_offset_angle) :
    _id_cam(id_cam),
    _tag(-1),
    _width(width),
    _height(height),
    _wc_offset_x(wc_offset_x),
    _wc_offset_y(wc_offset_y),
    _wc_offset_angle(wc_offset_angle),
    _expected_num_of_markers(0),
    _workerTh_running(0),
    _stop_workerTh(0)
{
    _resolution = height / wc_height;
    _sin_a = sin(wc_offset_angle);
    _cos_a = cos(wc_offset_angle);

    _greyMat = cv::Mat(_height, _width, CV_8UC1);

    pthread_mutex_init(&_mutexLocalization, NULL);
    pthread_mutex_init(&_mutexFrame, NULL);
}

Camera::~Camera(void)
{
    //pthread_mutex_destroy(&_mutexLocalization);
    //pthread_mutex_destroy(&_mutexFrame);
}

int Camera::get_id_cam()
{
    return _id_cam;
}

int Camera::get_height()
{
    return _height;
}

int Camera::get_width()
{
    return _width;
}

int Camera::get_tag()
{
    int copy;
    
    pthread_mutex_lock(&_mutexLocalization);
    copy = _tag;
    pthread_mutex_unlock(&_mutexLocalization);
    
    return copy;
}

std::vector<Marker> Camera::get_markers()
{
    std::vector<Marker> copy;

    pthread_mutex_lock(&_mutexLocalization);
    copy = _markers;
    pthread_mutex_unlock(&_mutexLocalization);

    return copy;
}

cv::Mat* Camera::get_frame()
{
    cv::Mat* frame;
    
    pthread_mutex_lock(&_mutexFrame);
    frame = new cv::Mat(_greyMat);
    pthread_mutex_unlock(&_mutexFrame);

    return frame;
}

void Camera::set_expected_num_of_markers(int n)
{
    _expected_num_of_markers = n;
}

class pixel{//se utiliza para guardar las cuatro esquinas.
public:
    float x;
    float y;
    pixel *next;

  
};




void* Camera::_localization_algorithm(void)
{
   
    cv::VideoCapture cap(_id_cam);
    if(!cap.isOpened()) {
        std::cerr << "Camera " << _id_cam << 
            " is already in use" << std::endl;
        pthread_exit(0);
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, _width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, _height);

    cv::Mat frame;

    int dictionaryId = 16;
    int smth_detected = 0;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
   
    for(;;){
        
        if(_stop_workerTh)
            pthread_exit(0);

        std::vector<Marker> local_markers;

        cap >> frame;

        pthread_mutex_lock(&_mutexFrame);
        cvtColor(frame, _greyMat, cv::COLOR_BGR2GRAY);
        pthread_mutex_unlock(&_mutexFrame);
        cap.retrieve(frame);
       
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        

        
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
		std::vector<std::vector<cv::Point2f>>::iterator row;
        std::vector<cv::Point2f>::iterator col;
                pixel p00;
                pixel p10;
                pixel p01;
                pixel p11;
                pixel *p=NULL;
                p=new pixel();
                p00.next=&p10;
                p10.next=&p01;
                p01.next=&p11;
                p11.next=NULL;
                p=&p00;
        // If at least one marker detected
        if ( ids.size()> 0)
        {
            
            cv::aruco::drawDetectedMarkers(_greyMat, corners, ids);
			//std::cout << corners.at(0).at(0).x<<std::endl;
            for(row=corners.begin(); row != corners.end();row++ )//hasta que ya no haya mas markers
            {   
                Marker marker;
                
                for(col=row->begin(); col != row->end();col++){
                    //hay que hacer una lista      
                    p->x=col->x;
                    p->y=col->y;
                    p=p->next;
                    
                }
                marker.id=reinterpret_cast<int>(ids.at(0));
                smth_detected = 1;
                marker.cam_corner_posX = static_cast<int>(p00.x);
                marker.cam_corner_posY = static_cast<int>(p00.y);

                uint16_t cx1 =static_cast<int> ((p10.x - p00.x)/2 + p00.x);
                uint16_t cx2 =static_cast<int> ((p11.x - p01.x)/2 + p01.x);
                uint16_t cy1 =static_cast<int> ((p01.y - p00.y)/2 + p00.y);
                uint16_t cy2 =static_cast<int> ((p11.y - p10.y)/2 + p10.y);

                marker.cam_center_posX = (cx1 + cx2)/2;
                marker.cam_center_posY = (cy1 + cy2)/2;

                // Angle in degress, w.r.t. the horizontal,
                // of the bottom solid border, counter-clockwise
                marker.cam_heading = atan2(p10.y - p00.y, p10.x - p00.x);

                marker.wc_corner_posX = marker.cam_corner_posX*_cos_a/_resolution +
                        marker.cam_corner_posY*_sin_a/_resolution + _wc_offset_x;
                marker.wc_corner_posY = -marker.cam_corner_posX*_sin_a/_resolution +
                        marker.cam_corner_posY*_cos_a/_resolution + _wc_offset_y;

                marker.wc_center_posX = marker.cam_center_posX*_cos_a/_resolution +
                        marker.cam_center_posY*_sin_a/_resolution + _wc_offset_x;
                marker.wc_center_posY = -marker.cam_center_posX*_sin_a/_resolution +
                        marker.cam_center_posY*_cos_a/_resolution + _wc_offset_y;

                marker.wc_heading = marker.cam_heading - _wc_offset_angle;

                local_markers.push_back(marker);
            }
           
           
        }
        
        
        imshow("Detected markers", _greyMat);
        
        char key = (char)cv::waitKey(10);
        if (key == 27)
            break;
        
        if(smth_detected)
        {
            smth_detected = 0;
            pthread_mutex_lock(&_mutexLocalization);
            _markers = local_markers;
            _tag++;
            pthread_mutex_unlock(&_mutexLocalization);
        }
    }
}

void * Camera::_localization_algorithm_helper(void * context)
{
    return ((Camera *)context)->_localization_algorithm();
}

void Camera::run(void)
{
   if(!_workerTh_running){
        _workerTh_running = 1;
        _stop_workerTh = 0;
        pthread_create(&_worker_thread, NULL,
                &Camera::_localization_algorithm_helper, this);
    }else
        std::cout << "Vision algorithm is already running for Camera " <<
            _id_cam << std::endl;
}

void Camera::stop(void)
{
    _stop_workerTh = 1;
    pthread_join(_worker_thread, NULL);
    _workerTh_running = 0;
}
