#include <ImFusion/Base/ImFusionFile.h>
#include "3d_livesegm_compounding/algorithm.h"
#include <ImFusion/US/UltrasoundSweep.h>           // for UltrasoundSweep
#include <ImFusion/Stream/TrackingStreamListener.h>
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/Stream/FakeImageStream.h>
#include <ImFusion/Stream/PlaybackTrackingStream.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/LiveUS/FreehandUSWorkflowAlgorithm.h>

#include <iostream>
#include <ImFusion/Base/DataModel.h>
#include <ImFusion/Stream/ImageStreamData.h>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/US/UltrasoundSweepRingBuffer.h>
#include <ImFusion/LiveUS/USSweepRecorderAlgorithm.h>
#include <ImFusion/LiveUS/USSweepRecorderController.h>


namespace ImFusion {
    namespace LiveSegmCompounding {

        bool PluginAlgorithm::createCompatible(const DataList &data, Algorithm **a) {
            if (data.size() != 0) { return false; }
            if (a) { *a = new PluginAlgorithm(); }
            return true;
        }

        void PluginAlgorithm::compute() {}

        void PluginAlgorithm::configure(const Properties *p) {
            if (p == nullptr) { return; }
            //p->param("something", something_);
        }

        void PluginAlgorithm::configuration(Properties *p) const {
            if (p == nullptr) { return; }
            //p->setParam("something", something_);
        }

        void PluginAlgorithm::run() {
        }

//        void PluginAlgorithm::imageCallBack() {
//
//        }

        void PluginAlgorithm::initSubscriber() {
            std::map<std::string, std::string> emptyArgs;
            if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
            ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
            ros_spinner_->start();

            // create a ROS handle to connect the neural network and the movement tracking parts
            ros::NodeHandle nh;
            // subscribers
            //this->segmentation_sub = nh.subscribe("/imfusion/sim_seg", 1, &PluginAlgorithm::subscribe_seg_result, this);
            this->segmentation_sub = nh.subscribe("/imfusion/sim_seg_s", 1, &PluginAlgorithm::subscribe_seg_result, this);

        }


        void PluginAlgorithm::subscribe_seg_result(sensor_msgs::ImageConstPtr img_msg) {
            cv_bridge::CvImagePtr cv_mask_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat gray;
            cv_mask_ptr->image.convertTo( gray, CV_8U, 255 );
            sendImageStream(gray);
//            m_segmented_images.push_back(cv_mask_ptr->image);
        }


        void PluginAlgorithm::sendImageStream(const cv::Mat& frame) {
            //stream_buffer_img = new TypedImage<unsigned char>(Image::Type::SHORT, frame.cols, frame.rows, 1, frame.channels());

            stream_buffer_img = std::make_shared<TypedImage<unsigned char>>(Image::Type::SHORT, frame.cols, frame.rows, 1, frame.channels());

            auto sharedImage = ImFusion::SharedImage(stream_buffer_img);

            std::shared_ptr<ImFusion::SharedImage> shared_image_ptr = std::make_shared<ImFusion::SharedImage>(sharedImage);
            

            spacing_mutex.lock();
            stream_buffer_img->setSpacing(m_spacing[0], m_spacing[1]);
            spacing_mutex.unlock();

            memcpy(stream_buffer_img->data(), frame.data,
                   sizeof(unsigned char) * uint64(frame.rows * frame.cols * frame.channels()));
            std::chrono::system_clock::time_point arrivalTime = std::chrono::high_resolution_clock::now();

            ImFusion::ImageStreamData streamImage(current_image_stream, shared_image_ptr);
            streamImage.setTimestampArrival(arrivalTime);

            current_image_stream->setModality(Stream::Modality::ULTRASOUND);
            current_image_stream_out->sendStreamData(streamImage);
            //delete stream_buffer_img;
        }



        void PluginAlgorithm::startDummyStream(MainWindowBase *main) {
//            ImFusionFile file("/media/dani/Expansion/0_IFL/AAA_aorta_scan/Felix-02.imf");
//            file.open(0);
//
//            DataList dataList;
//            file.load(dataList);
//            auto usSweep = dynamic_cast<UltrasoundSweep*>(dataList.getItem(0));
//            auto components = usSweep->components();
//
//            LiveTrackingStream *robStream;
//            ImageStream *imageStream = new FakeImageStream();
//            imageStream->open();
//            imageStream->start();
//            robStream = new FakeTrackingStream();
//////            usStream = static_cast<ImageStream *>(main->dataModel()->get("liveSegmentationStream"));
//            robStream->open();
//            robStream->start();
//            std::vector<Stream *> vec;
//            vec.push_back((imageStream));
//            vec.push_back((robStream));
//
//            USSweepRecorderAlgorithm *sweepRecorderAlgorithm = new USSweepRecorderAlgorithm(vec);
//            USSweepRecorderController *mySweepController = new USSweepRecorderController(
//                    sweepRecorderAlgorithm);
//////            mySweepController->init();
//            sweepRecorderAlgorithm->start();
//            sweepRecorderAlgorithm->recorder();
//
//            auto sweepBuffer = mySweepController->liveSweep();
//            sweepBuffer->setBufferSize(100);
//            main->dataModel()->add(*sweepRecorderAlgorithm->trackingStream(), "live stream");


        }

    }  // namespace LiveSegmCompounding
}  // namespace ImFusion
