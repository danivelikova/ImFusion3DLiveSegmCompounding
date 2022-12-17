#pragma once

#include <ImFusion/Base/Algorithm.h>

#include <QObject>
#include <QtCore/QThread>

#include <string>

#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Base/TypedImage.h>
#include <QTimer>
#include <ImFusion/Stream/PlaybackStream.h>
#include <ImFusion/GUI/MainWindowBase.h>
//#include <ImFusion/Cephasonics/CephasonicsStream.h>
//#include <ImFusion/ROS/Stream/ROSTopicImageOutStream.h>
#include <ImFusion/Stream/OpenIGTLinkImageStream.h>  // for OpenIGTLinkImage...
#include <ImFusion/Stream/OpenIGTLinkImageOutStream.h>  // for OpenIGTLinkImage...
// #include <ImFusion/Stream/OpenIGTLinkStreamData.h>   // for OpenIGTLinkImage...
#include <ImFusion/Stream/StreamData.h>
#include <ImFusion/Stream/ImageStreamData.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <ImFusion/Stream/SetImageStreamModality.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>


namespace ImFusion {
    namespace LiveSegmCompounding {

//class PluginAlgorithm : public QObject, public ImFusion::Algorithm {
        class PluginAlgorithm : public QThread, public ImFusion::Algorithm {
        Q_OBJECT
        public:
            PluginAlgorithm() = default;

            ~PluginAlgorithm() = default;

            void compute();

            void run() override;

            void preprocess_Image(ImageStream *usStream);

            void startDummyStream(MainWindowBase* main);

            //! Methods implementing the Configurable interface.
            void configure(const Properties *p);

            void configuration(Properties *p) const;

            static bool createCompatible(const DataList &data, Algorithm **a = nullptr);

            void subscribe_seg_result(sensor_msgs::ImageConstPtr img_msg);

            void sendImageStream(const cv::Mat& frame);
            void initSubscriber();


        public:
//            void imageCallBack();

            std::shared_ptr<TypedImage<unsigned char>> stream_buffer_img;
            OpenIGTLinkImageStream* current_image_stream;
            OpenIGTLinkImageOutStream* current_image_stream_out;
            std::unique_ptr<ros::AsyncSpinner> ros_spinner_{nullptr};
            std::vector<cv::Mat> m_segmented_images; // a vector to collect all the segmented US images.
            ros::Subscriber segmentation_sub;


        private:
            ImageStream *usStream;
            QTimer *timer;
            PlaybackStream *playbackStream;


            std::mutex spacing_mutex;
            std::vector<double> m_spacing{0.5, 0.5};

//  ROSTopicImageOutStream* rosOutStr;

        protected:
            bool run_{true};

        };
    }  // namespace LiveSegmCompounding
}  // namespace ImFusion
