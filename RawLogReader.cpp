/*
 * RawLogReader.cpp
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
 */

#include "RawLogReader.h"

RawLogReader::RawLogReader(Bytef *& decompressionBuffer,
                           IplImage *& deCompImage,
                           std::string file,
                           bool flipColors)
 : decompressionBuffer(decompressionBuffer),
   deCompImage(deCompImage),
   file(file),
   flipColors(flipColors)
{
    assert(boost::filesystem::exists(file.c_str()));

    fp = fopen(file.c_str(), "rb");

    currentFrame = 0;

    assert(fread(&numFrames, sizeof(int32_t), 1, fp));

    depthReadBuffer = new unsigned char[Resolution::getInstance().numPixels() * 2];
    imageReadBuffer = new unsigned char[Resolution::getInstance().numPixels() * 3];
}

RawLogReader::~RawLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;

    fclose(fp);
}

void RawLogReader::getNext()
{
    assert(fread(&timestamp, sizeof(int64_t), 1, fp));

    assert(fread(&depthSize, sizeof(int32_t), 1, fp));
    assert(fread(&imageSize, sizeof(int32_t), 1, fp));

    assert(fread(depthReadBuffer, depthSize, 1, fp));

    if(imageSize > 0)
    {
        assert(fread(imageReadBuffer, imageSize, 1, fp));
    }

    unsigned long decompLength = Resolution::getInstance().numPixels() * 2;
    uncompress(&decompressionBuffer[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);

    if(deCompImage != 0)
    {
        cvReleaseImage(&deCompImage);
    }

    CvMat tempMat = cvMat(1, imageSize, CV_8UC1, (void *)imageReadBuffer);

    if(imageSize > 0)
    {
        deCompImage = cvDecodeImage(&tempMat);
    }
    else
    {
        deCompImage = cvCreateImage(cvSize(Resolution::getInstance().width(), Resolution::getInstance().height()), IPL_DEPTH_8U, 3);
        memset(deCompImage->imageData, 0, Resolution::getInstance().numPixels() * 3);
    }

    if(flipColors)
    {
        cv::Mat3b rgb(Resolution::getInstance().rows(), Resolution::getInstance().cols(), (cv::Vec<unsigned char, 3> *)deCompImage->imageData, Resolution::getInstance().width() * 3);
        cv::cvtColor(rgb, rgb, CV_RGB2BGR);
    }

    currentFrame++;
}

int RawLogReader::getNumFrames()
{
    return numFrames;
}

bool RawLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}

