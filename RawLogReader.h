/*
 * RawLogReader.h
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
 */

#ifndef RAWLOGREADER_H_
#define RAWLOGREADER_H_

#include <cassert>
#include "Utils/Resolution.h"
#include <zlib.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <boost/filesystem.hpp>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif

class RawLogReader
{
    public:
        RawLogReader(Bytef *& decompressionBuffer,
                     IplImage *& deCompImage,
                     std::string file,
                     bool flipColors);

        virtual ~RawLogReader();

        void getNext();

        int getNumFrames();

        bool hasMore();

        Bytef *& decompressionBuffer;
        IplImage *& deCompImage;
        unsigned char * depthReadBuffer;
        unsigned char * imageReadBuffer;
        int64_t timestamp;
        int32_t depthSize;
        int32_t imageSize;

    private:
        const std::string file;
        FILE * fp;
        int32_t numFrames;
        int currentFrame;
        bool flipColors;
};


#endif /* RAWLOGREADER_H_ */
