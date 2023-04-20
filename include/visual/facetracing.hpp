#ifndef __FACETRACING_H__
#define __FACETRACING_H__

#include <list>
#include <opencv2/core.hpp>

void ftFindFace(cv::Mat &img, const char * name);

void fioReadFaceFeature();
void fioSaveFaceFeature(cv::Mat &img, const char * name);

void freeFeatureList();

void ftSetAwake(int awake);

void ftSetFinish(int finish);

void ftCalculateDepth(cv::Mat &img, int argc, char ** argv, int &step);

#endif /* __FACETRACING_H__ */