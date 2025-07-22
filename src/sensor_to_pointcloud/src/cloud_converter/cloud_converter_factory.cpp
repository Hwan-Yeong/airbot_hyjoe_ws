#include "cloud_converter/cloud_converter_factory.hpp"
#include "cloud_converter/cloud_converter.hpp"

namespace sensor_to_pointcloud {

CloudConverterPtr CloudConverterFactory::create(SensorType type, std::shared_ptr<SensorToPointcloudNode> node_ptr) {
    switch(type) {
        case SensorType::Camera:
            return std::make_shared<CameraCloudConverter>(node_ptr);
        default:
            return nullptr;
    }
}

}
