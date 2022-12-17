#include "3d_livesegm_compounding/sweep_rec_and_comp.h"
#include <ImFusion/Base/Log.h>
#include <ImFusion/GL/GlVolumeCompounding.h>
#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/GL/SharedImageSet.h> // for SharedImageSet
#include <ImFusion/IO/BackgroundExporter.h>
#include <ImFusion/Base/DataModel.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <iostream>
#include <chrono>
//#include <ImFusion/LiveUS/USSweepRecorderAlgorithm.h>
#include <ImFusion/LiveUS/USSweepRecorderController.h>
#include <ImFusion/LiveUS/LiveSweepRecordingVisualizationController.h>
#include <ImFusion/US/UltrasoundSweepRingBuffer.h>
#include <ImFusion/Base/BasicImageProcessing.h>
#include <ImFusion/GL/IntensityMask.h>



#define ON_NOTREADY -1
#define ON_READYTOSTART 0
#define ON_FINAL 1

namespace ImFusion {
    namespace LiveSegmCompounding {

        SweepRecAndComp::SweepRecAndComp(MainWindowBase *mainWindowBase)
                : m_main(mainWindowBase ) {
        }

        void SweepRecAndComp::startSweepRecording() {
            LiveTrackingStream *robStream;
            ImageStream *usStream;

            if (m_useLiveData) {
                LOG_INFO("m_useLiveData: ", m_useLiveData);

                robStream = static_cast<LiveTrackingStream *>(m_main->dataModel()->get(
                        "RobotTrackingStream"));
                usStream = static_cast<ImageStream *>(m_main->dataModel()->get("liveSegmentationStream"));
            } else {
                robStream = new FakeTrackingStream(FakeTrackingStream::OscillatingDevice(), "Fake Device");
                usStream = new FakeImageStream();
                robStream->open();
                robStream->start();
                usStream->open();
                usStream->start();

            }
            LOG_INFO("setModality ");

            usStream->setModality(Data::Modality::ULTRASOUND);
            std::vector<Stream *> vec;
            vec.push_back((usStream));
            vec.push_back((robStream));
//            ImageStream * USStream = static_cast<ImageStream *>(m_main->dataModel()->get("Ultrasound Stream"));
            // auto cepha_stream = m_main->dataModel()->get("Cephasonics Ultrasound");
            // auto stream = static_cast<DataGroup *>(cepha_stream)->at(0);
            // ImageStream* USStream = static_cast<ImageStream*>(stream);
            // vec.push_back((USStream));

            LOG_INFO("sweepRecorderAlgorithm starting");
            sweepRecorderAlgorithm = new USSweepRecorderAlgorithm(vec);
            sweepRecorderAlgorithm->start();
            Properties ctrlRecorder;
            ctrlRecorder.addSubProperties("Controller");
            m_main->addAlgorithm(sweepRecorderAlgorithm, &ctrlRecorder);
            
            USSweepRecorderController *controller = dynamic_cast<USSweepRecorderController *>(m_main->getAlgorithmController(sweepRecorderAlgorithm));
            ringBuffer = controller->liveSweep();
            ringBuffer->setBufferSize(999999999);



//            IntensityMask intensityMask = new IntensityMask();
//            ringBuffer->setMask()
//            fanMotionTimer = new QTimer(this);
//            connect(fanMotionTimer, SIGNAL(timeout()), this, SLOT(onUpdateVolume()));
//            fanMotionTimer->start(15000);
        }

        void SweepRecAndComp::onUpdateVolume() {
            LOG_INFO("INSIDE onUpdateVolume ");

            numberOfPartialSweeps += 1;
            DataList datalist;
            sweepRecorderAlgorithm->stop();

            sweepRecorderAlgorithm->output(datalist);
//            sweepRecorderAlgorithm->start();

            UltrasoundSweep *usSweep = static_cast<UltrasoundSweep *>(datalist.getItem(0));

            // UltrasoundSweep *usSweepOriginal = static_cast<UltrasoundSweep *>(datalist.getItem(1));

            // necessary???
//               usSweep->tracking()->setTemporalOffset(146);
            Selection sel;
            // Selection sel2;
            LOG_INFO("sweepRecorderAlgorithm stop ");

            sel.setAll(usSweep->size());
            // sel.setAll(usSweepOriginal->size());
            LOG_INFO("sweepRecorderAlgorithm stop ");

            usSweep->setSelection(sel);
            // usSweepOriginal->setSelection(sel2);
            LOG_INFO("sweepRecorderAlgorithm stop ");

//            sel.setAll(ringBuffer->size());
//            ringBuffer->setSelection(sel);
//             usSweep->tracking()->setTemporalOffset(m_robotCtrlUS->getTemporalCalibration());
            // setConvexGeometry(usSweep);

            auto *sc2 = new GlSweepCompounding(*usSweep);
            LOG_INFO("sweepRecorderAlgorithm stop ");

            sc2->setMode(0); // GPU    4 is CPU Maximum
            sc2->compute();  // We do the volume compounding
            DataList datalist3;
            sc2->output(datalist3);

            m_main->dataModel()->add(datalist3.getItem(0), "Volume ");
            LOG_INFO("sweepRecorderAlgorithm stop ");

//            m_main->dataModel()->add(usSweep, "Partial Sweep ");

            if (m_exportSweeps) {
                LOG_INFO("m_exportSweeps ");

                auto sis = static_cast<SharedImageSet *>(datalist3.getItem(0));
                //sis->get()->sync();
                auto str = getDayAndTime();
                BackgroundExporter *sweepExporter = new BackgroundExporter();
                ImFusion::DataList data;
                data.add(usSweep);
                sweepExporter->saveBlocking(data, "/home/aorta-scan/dani/Aorta_scan_Live3D/results_sweeps/sweep_" + str + ".imf");
                // BackgroundExporter *originalSweepExporter = new BackgroundExporter();
                // ImFusion::DataList data2;
                // data2.add(usSweepOriginal);
                // originalSweepExporter->saveBlocking(data2, "/media/dani/Expansion/0_IFL/Aorta_scan_Live3D/results_sweeps/original_sweep_" + str + ".imf");
                
                // BackgroundExporter *volExporter = new BackgroundExporter();
                //  ImFusion::DataList data3;
                // data3.add(usSweepOriginal);
                // volExporter->saveBlocking(data3, "/media/dani/Expansion/0_IFL/Aorta_scan_Live3D/results_sweeps/vol_" + str + ".imf");
            }

            delete sc2;
        }

        void SweepRecAndComp::stop() {
            onUpdateVolume();
//            /?fanMotionTimer->stop();
//            delete sweepRecorderAlgorithm;
        }


        std::string SweepRecAndComp::getDayAndTime() {
            time_t rawtime;
            struct tm *timeinfo;
            char buffer[80];

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            strftime(buffer, sizeof(buffer), "%d_%m_%H_%M_%S", timeinfo);
            std::string str(buffer);

            return str;
        }
    } // namespace vision_control
} // namespace ImFusion
