#include "3d_livesegm_compounding/controller.h"
#include "3d_livesegm_compounding/algorithm.h"
#include <QDebug>
#include <ImFusion/Base/Log.h>
#include <ImFusion/Base/DataModel.h>

#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>


#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/Base/ImFusionFile.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/Stream/FakeImageStream.h>


#include <iostream>

#undef slots

//#include <torch/script.h> // One-stop header

#define slots Q_SLOTS

#include "ui_controller.h"
#include <opencv2/opencv.hpp>
#include <ImFusion/Stream/OpenIGTLinkConnection.h>



const bool useDummyData = false;
const bool performSegmentationAndCompound = true;

namespace ImFusion {
    namespace LiveSegmCompounding {

        PluginController::PluginController(PluginAlgorithm *algorithm)
                : AlgorithmController(algorithm), algorithm_{algorithm} {
            ui_ = std::make_shared<Ui_Controller>();
            ui_->setupUi(this);
            //qRegisterMetaType<cv::Mat>("cvMat");
        }


        void PluginController::init() {
            addToAlgorithmDock();
            showMinimized();

            // algorithm_->initSubscriber();


            connect(ui_->pbtStartSweep, &QPushButton::clicked, this, &PluginController::onStartClicked);
            connect(ui_->pbtStopSweep, &QPushButton::clicked, this, &PluginController::onStopClicked);
            connect(ui_->pbtConnectRobot, &QPushButton::clicked, this, &PluginController::onConnectRobotClicked);
            connect(ui_->pbtSegmentation, &QPushButton::clicked, this, &PluginController::onStartSegmentationClicked);

            sweepRecAndComp = new SweepRecAndComp(m_main);

        }

        void PluginController::addToDataModel(ImFusion::Data* data, std::string name) { m_main->dataModel()->add(data, name); }


        void PluginController::onStartClicked() {
           sweepRecAndComp->startSweepRecording();
        }

        void PluginController::onStopClicked() {
//            imageStream->stop();
//            imageStream->close();
            sweepRecAndComp->stop();
//            robControl->applyPositionControlMode();

        }

        void PluginController::onConnectRobotClicked() {

            robotTrackingStream = new CustomROSTopicTrackingStream();
            robotTrackingStream->start();
            m_main->dataModel()->add(robotTrackingStream, "RobotTrackingStream");
            LOG_INFO("RobotTrackingStream created: ", robotTrackingStream->isRunning());

//            robot_in = new ROSTopicTrackingStream("ROS Topic Tracking Stream", "/iiwa/state/CartesianPose");
//            robot_in->start();
//            m_main->dataModel()->add(robot_in, "RobotTrackingStream");
//            LOG_INFO("RobotTrackingStream created: ", robot_in->isRunning());


//            loadCalibrationFromFile("IFLUSCalibration.config", probe_name);
        }


        void PluginController::onStartSegmentationClicked() {
            segimageStream = new ROSTopicImageStream("ROS Topic Image Stream", "/imfusion/sim_seg_s");
            //segimageStream->open();
            m_main->dataModel()->add(segimageStream, "liveSegmentationStream");
            segimageStream->start();

            LOG_INFO("ROSTopicImageStream created: ", segimageStream->isRunning());

        }
////            if (useDummyData) {
////                ImFusionFile file(
////                        "/media/dani/Expansion/0_IFL/AAA_aorta_scan/original_sweep_31_05_12_33_20.imf");
////                file.open(0);
////
////                DataList dataList;
////                file.load(dataList);
////
////                SharedImageSet *sis = dataList.getImage(Data::UNKNOWN);
////                imageStream = new PlaybackStream(*sis);
////            } else {
//////                imageStream = static_cast<ImageStream *>(m_main->dataModel()->get("Ultrasound Stream"));
////
////
////            }
//
////            imageStream = new FakeImageStream
//            LOG_INFO("create ROSTopicImageStream");
//
//
//            LOG_INFO("add to liveSegmentationStream");
////            StreamListener* usStream;
////            robimageStream->addListener(usStream);

//
////            usStream->start();
////            start();
//
////            liveSegmentationStream->addListener(robimageStream);
//            //rtos->start();
//
//        }

    }  // namespace LiveSegmCompounding
}  // namespace ImFusion
