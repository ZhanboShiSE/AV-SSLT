#include <iostream>
#include <string>
#include <fstream>
#include <list>
#include <memory>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "image_ros_read.hpp"

using namespace std;
using namespace cv;


class FaceFeature {
    public:
        FaceFeature();
        FaceFeature(string m_name, string m_feature);
        string ff_get_name();
        string ff_get_feature();
        void ff_set_name(string m_name);
        void ff_set_feature(string m_feature);
    private:
        string name;
        string feature;
};

FaceFeature::FaceFeature() {
    this->name = "";
    this->feature = string();
}
FaceFeature::FaceFeature(string m_name, string m_feature): name(m_name), feature(m_feature) {}

string FaceFeature::ff_get_name() { return this->name; }
string FaceFeature::ff_get_feature() { return this->feature; }

void FaceFeature::ff_set_name(string m_name) { this->name = m_name; }
void FaceFeature::ff_set_feature(string m_feature) { this->feature = m_feature; }


static const int FACE_SCALE              = 32;
static const int FACE_NUM_LIMIT          = 10;
static const char * FACE_FEATURE_DATA_PATH  = "./resource/face_feature_data.txt";
static const int ID_LIST_MAX_SIZE = 10


static handle ft_handle = nullptr;
static list<string> ft_detected_multiface = {0};
static list<int> ft_id_list;

static int ft_focused_face_x1 = 0;
static int ft_focused_face_x2 = 100;
static int ft_focused_face_y1 = 0;
static int ft_focused_face_y2 = 100;

static list<FaceFeature> fio_feature_list;

static int ft_is_awake = 0;
static int ft_is_finish = 1;


Mat ft_cut_video_image(Mat src) {
    Mat dst, mid;
    resize(src, mid, Size(src.cols - src.cols%4, src.rows - src.rows%4));
    int length = (mid.cols > mid.rows) ? mid.rows : mid.cols;
    mid = mid(Range(0, length), Range(0, length));
    dst = mid.clone();
    return dst;
}


void ftFindFace(Mat &img, const char * name) {
    img = ft_cut_video_image(img);
    
    ft_detect_face(img);
    
    if (ft_detected_multiface.faceNum > 0) {
        for (int i = 0; i < ft_detected_multiface.faceNum; i++) {
            int x1, x2, y1, y2;
            x1 = (ft_detected_multiface.faceRect[i].left < 0) ? 0 : ft_detected_multiface.faceRect[i].left;
            x2 = (ft_detected_multiface.faceRect[i].right > img.cols) ? img.cols : ft_detected_multiface.faceRect[i].right;
            y1 = (ft_detected_multiface.faceRect[i].top < 0) ? 0 : ft_detected_multiface.faceRect[i].top;
            y2 = (ft_detected_multiface.faceRect[i].bottom > img.rows) ? img.rows : ft_detected_multiface.faceRect[i].bottom;
            rectangle(img, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255));

            list<int>::iterator id_pos = ::find(ft_id_list.begin(), ft_id_list.end(), ft_detected_multiface.faceID[i]);
            if (id_pos != ft_id_list.end()) {
                ft_focused_face_x1 = x1;
                ft_focused_face_x2 = x2;
                ft_focused_face_y1 = y1;
                ft_focused_face_y2 = y2;
                continue;
            }
            
            else {
                while (ft_id_list.size() > ID_LIST_MAX_SIZE)
                    ft_id_list.pop_front();
                
                ft_id_list.push_back(ft_detected_multiface.faceID[i]);
                string feature = {0};
                FaceInfo info = {0};
                info.faceRect = ft_detected_multiface.faceRect[i];
                info.faceOrient = ft_detected_multiface.faceOrient[i];
                FaceFeature face_feature = ft_extract_feature(info, feature, img, "");

                float level = 0.0f;
                string img_feature, saved_feature;
                img_feature.feature = face_feature.ff_get_feature().feature;
                img_feature.featureSize = face_feature.ff_get_feature().featureSize;

                list<FaceFeature>::iterator iter, face_pos = fio_feature_list.end();
                LPstring feature1, feature2;
                feature1 = &img_feature;

                for (iter = fio_feature_list.begin(); iter != fio_feature_list.end(); iter++) {
                    saved_feature.feature = iter->ff_get_feature().feature;
                    saved_feature.featureSize = iter->ff_get_feature().featureSize;
                    feature2 = &saved_feature;

                    ft_feature_compare(feature1, feature2, level);
                    if (level >= 0.8f && iter->ff_get_name() == name) {
                        face_pos = iter;
                        ft_focused_face_x1 = x1;
                        ft_focused_face_x2 = x2;
                        ft_focused_face_y1 = y1;
                        ft_focused_face_y2 = y2;
                        break;
                    }
                }

                if (face_pos != fio_feature_list.end()) {
                    cout << "Find Face! Name: " << face_pos->ff_get_name() << ", Compare Level: " << level << endl;

                }
                else 
                    cout << "Face Not Find With Name: " << name << endl;
            }
        }
    }
}


void fioReadFaceFeature() {
    if (!fio_feature_list.empty()) 
        fio_feature_list.clear();
    
    ifstream fp;
    fp.open(FACE_FEATURE_DATA_PATH, std::ios::in);
    if (!fp.is_open()) {
        cout << "Face Feature Data Read Failed!" << endl;
        return;
    }
    
    while (!fp.eof()) {
        string name = "";
        signed int length = 0;
        fp >> name >> length;
        
        unsigned short feature_converted_to_short[length];
        unsigned char * feature_converted_to_byte = new unsigned char[length];
        for (int i = 0; i < length; i++) {
            fp >> feature_converted_to_short[i];
            feature_converted_to_byte[i] = (unsigned char)feature_converted_to_short[i];
        }

        string m_feature = {0};
        m_feature.feature = feature_converted_to_byte;
        m_feature.featureSize = length;

        FaceFeature face_feature = FaceFeature(name, m_feature);
        fio_feature_list.push_back(face_feature);

        feature_converted_to_byte = nullptr;
    }

    fp.close();
    cout << "Face Feature Read Successful" << endl;
}


void fioSaveFaceFeature(Mat &img, const char * name) {
    img = ft_cut_video_image(img);

    ft_detect_face(img);

    FaceInfo info = {0};
    info.faceOrient = ft_detected_multiface.faceOrient[0];
    info.faceRect = ft_detected_multiface.faceRect[0];

    string feature = {0};
    ft_extract_feature(info, feature, img, name);

    ofstream fp;
    fp.open(FACE_FEATURE_DATA_PATH, std::ios::app);
    if (!fp.is_open()) {
        cout << "Face Feature Data Write Failed!" << endl;
        return;
    }
    
    fp << endl << name << ' ' << feature.featureSize << ' ';
    for (int i = 0; i < feature.featureSize; i++) {
        fp << (unsigned short)feature.feature[i] << ' ';
    }
    
    fp.close();
}


void freeFeatureList() {
    list<FaceFeature>::iterator iter;
    for (iter = fio_feature_list.begin(); iter != fio_feature_list.end(); iter++) {
        string m_feature = {0}, null_feature = {0};
        m_feature.feature = iter->ff_get_feature().feature;
        delete []m_feature.feature;
        iter->ff_set_feature(null_feature);
    }

    while (!fio_feature_list.empty())
        fio_feature_list.pop_front();
}


void ftSetAwake(int awake) {
    ft_is_awake = awake;
}


void ftSetFinish(int finish) {
    ft_is_finish = finish;
}


void ftCalculateDepth(Mat &img, int argc, char ** argv, int &step) {
    img = ft_cut_video_image(img);

    if (ft_is_awake == 1 && ft_is_finish == 0) {
        int width = (ft_focused_face_x2 - ft_focused_face_x1) / 2;
        int height = (ft_focused_face_y2 - ft_focused_face_y1) / 2;
        int x = ft_focused_face_x1 + width / 2;
        int y = ft_focused_face_y1 + height / 2;
        
        Mat face_img = img(Rect(x, y, width, height)).clone();
        cv::Mat tmp = face_img.reshape(0, 1);
        Mat sorted;
        cv::sort(tmp, sorted, SORT_ASCENDING);
        
        uint16_t d = sorted.at<uint16_t>(sorted.cols / 2);
        float fd = d;
        fd /= 1000;

        x = (ft_focused_face_x1 + ft_focused_face_x2) / 2;
        int x_left, x_right, rotation = 0;
        x_left = img.rows / 5 * 3;
        x_right = img.rows - img.rows / 5;
        if (x <= x_left)
            rotation = -1;
        else if (x >= x_right)
            rotation = 1;

        if (fd > 0.5) {
            step += 1;
            msgMoveRosPublish(argc, argv, rotation);
        }
        else {
            ft_is_awake = 0;
            ft_is_finish = 1;
            step = 0;
        }
    }

    // the depth sensor work in range 0.4~8m
    convertScaleAbs(img, img, 0.03);
    applyColorMap(img, img, COLORMAP_JET);
    rectangle(img, Point(ft_focused_face_x1, ft_focused_face_y1), Point(ft_focused_face_x2, ft_focused_face_y2), Scalar(0, 255, 255), 1, 8, 0);
    imshow("depth", img);
}