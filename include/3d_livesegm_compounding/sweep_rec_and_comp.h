#pragma once

#include <Eigen/Dense>
#include <QtCore/QObject>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/LiveUS/USSweepRecorderAlgorithm.h>
#include <ImFusion/Base/DataList.h>
#include <QtCore/QThread>
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/Stream/FakeImageStream.h>
#include <ImFusion/US/UltrasoundSweepRingBuffer.h>

namespace ImFusion {
    namespace LiveSegmCompounding {

        class SweepRecAndComp : public QObject {
        Q_OBJECT
        public:
            explicit SweepRecAndComp(MainWindowBase *mainWindowBase);

            void startSweepRecording();

            void stop();

            std::string getDayAndTime();

        public slots:

            void onUpdateVolume();


        private:
            MainWindowBase *m_main{nullptr};
            QTimer *timer;
            bool m_useLiveData = true;
            bool m_exportSweeps = true;
            int numberOfPartialSweeps{0};
            DataList m_dataList;
            UltrasoundSweepRingBuffer *ringBuffer;
            USSweepRecorderAlgorithm *sweepRecorderAlgorithm{};


        };


    }  // namespace vision_control
}  // namespace ImFusion
