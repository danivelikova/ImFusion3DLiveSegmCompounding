#pragma once

#include <ImFusion/GUI/AlgorithmController.h>
#include <QtWidgets/QWidget>

#include <memory>
// /************************************************************************
//  *                           ImFusion
//  * **********************************************************************/
// #include <ImFusion/Base/Algorithm.h>  // for IoAlgorithm
// #include <ImFusion/Base/DataList.h>   // for DataList
// #include <ImFusion/Base/AlgorithmListener.h>  // for AlgorithmListener
// #include <ImFusion/Base/Log.h>                // for LOG_INFO
// #include <ImFusion/Base/Properties.h>         // for Properties
// #include <ImFusion/Base/MemImage.h>
// #include <ImFusion/Base/SharedImage.h>
// #include <ImFusion/Base/SharedImageSet.h>
// #include <ImFusion/Base/Progress.h>
// #include <ImFusion/Base/OpenCV.h>
// #include <ImFusion/Base/ImFusionFile.h>
// #include <ImFusion/Base/TypedImage.h>
// #include <ImFusion/Base/TrackingStream.h>
// #include <ImFusion/GL/GlVolumeCompounding.h>
// #include <ImFusion/GL/GlSliceView.h>
// #include <ImFusion/GUI/AnnotationModel.h>
// #include <ImFusion/GUI/DataWidget.h>
// #include <ImFusion/GUI/DisplayWidgetMulti.h>
// #include <ImFusion/GUI/ImageView2D.h>
// #include <ImFusion/GUI/ImageView3D.h>
// #include <ImFusion/GUI/Interactive.h>
// #include <ImFusion/GUI/MainWindowBase.h>
// #include <ImFusion/GUI/AlgorithmController.h>
// #include <ImFusion/IO/BackgroundExporter.h>
// #include <ImFusion/Stream/OpenIGTLinkStreamData.h>
// #include <ImFusion/Stream/OpenIGTLinkImageStream.h>
// #include <ImFusion/Stream/LiveTrackingStream.h>
// #include <ImFusion/Stream/TrackingStreamData.h>
// #include <ImFusion/US/UltrasoundSweep.h>
// #include <ImFusion/US/USConfidenceMapAlgorithm.h>
// #include <ImFusion/US/FanMask.h>
// #include <ImFusion/US/UltrasoundGeometry.h>
// #include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/LiveUS/USSweepRecorderAlgorithm.h>
// #include <ImFusion/US/USSweepCalibrator.h>

// /************************************************************************
//  *                           Qt
//  * **********************************************************************/
// #include <QtWidgets/QWidget>
// #include <QtDebug>
// #include <QFile>
// #include <QTextStream>
// #include <QDir>
// #include <memory>
// #include <QtCore/QObject>
// #include <qt5/QtNetwork/QTcpServer>  //for TCP
// #include <qt5/QtNetwork/QTcpSocket>  //for TCP
// #include <QTimer>
// #include <QTime>
// #include <qmutex.h>
// #include <unistd.h>
// #include <functional>
//#include "3d_livesegm_compounding/RobotControl.h"
#include "sweep_rec_and_comp.h"
#include "CustomROSTopicTrackingStream.h"
#include <ImFusion/ROS/Stream/ROSTopicImageStream.h>
#include <ImFusion/ROS/Stream/ROSTopicTrackingStream.h>


class Ui_Controller;
// Q_DECLARE_METATYPE(cv::Mat)

namespace ImFusion {
    namespace LiveSegmCompounding {

        class PluginAlgorithm;

        class PluginController : public QWidget, public AlgorithmController {
        Q_OBJECT
        public:
            /// Constructor with the algorithm instance
            PluginController(PluginAlgorithm *algorithm);

            virtual ~PluginController() = default;

            void init();
            // void startRecording();

            QTimer *timer;
        signals:

            void start();

            void stop();

        public slots:

            void onStartClicked();

            void onStopClicked();

            void onConnectRobotClicked();

            void onStartSegmentationClicked();

            void addToDataModel(ImFusion::Data* data, std::string name);

        private:

            std::shared_ptr<Ui_Controller> ui_{nullptr}; ///< The actual GUI
            PluginAlgorithm *algorithm_{nullptr};        ///< The algorithm instance
            SweepRecAndComp *sweepRecAndComp;
            ImageStream *imageStream;
            CustomROSTopicTrackingStream* robotTrackingStream {nullptr};
//            ROSTopicTrackingStream* robot_in{nullptr};
            ROSTopicImageStream* segimageStream {nullptr};



        };

    } // namespace LiveSegmCompounding
} // namespace ImFusion
