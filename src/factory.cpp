#include "3d_livesegm_compounding/factory.h"
#include "3d_livesegm_compounding/algorithm.h"
#include "3d_livesegm_compounding/controller.h"

namespace ImFusion {
    namespace LiveSegmCompounding {

        PluginAlgorithmFactory::PluginAlgorithmFactory() {
            registerAlgorithm<PluginAlgorithm>("ROS;LiveSegmCompounding");
        }

        AlgorithmController *PluginControllerFactory::create(ImFusion::Algorithm *a) const {
            if (PluginAlgorithm * algorithm = dynamic_cast<PluginAlgorithm *>(a)) {
                return new PluginController(algorithm);
            }
            return nullptr;
        }

    }  // namespace LiveSegmCompounding
}  // namespace ImFusion
