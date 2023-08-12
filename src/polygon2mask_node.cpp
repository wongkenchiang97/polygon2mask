#include <polygon2mask/polygon2mask.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polygon2mask_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    const std::string dataset_dir = "/home/dylan/Documents/datasets";
    const std::string uri = "mongodb://localhost:3001";
    auto polygon2mask = image_base::Polygon2Mask::create(uri, dataset_dir);

    ros::waitForShutdown();

    return 0;
}