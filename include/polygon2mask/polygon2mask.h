#ifndef POLYGON_2_MASK_H
#define POLYGON_2_MASK_H

#include <bsoncxx/v_noabi/bsoncxx/json.hpp>
#include <memory>
#include <mongocxx/v_noabi/mongocxx/client.hpp>
#include <mongocxx/v_noabi/mongocxx/instance.hpp>
#include <polygon2mask/image_label.h>
#include <ros/ros.h>

namespace image_base {
class Polygon2Mask {
public:
    using Ptr = std::shared_ptr<Polygon2Mask>;

    Polygon2Mask(const std::string& _uri, const std::string& _dataset_dir);
    virtual ~Polygon2Mask();
    static Polygon2Mask::Ptr create(const std::string& _uri, const std::string& _dataset_dir);

private:
    mongocxx::instance inst_;
    mongocxx::client client_;
    std::vector<ImageAnnotation::Ptr> image_annotations_;
    std::string dataset_dir_;
};

Polygon2Mask::Polygon2Mask(const std::string& _uri, const std::string& _dataset_dir)
    : inst_(mongocxx::instance {})
    , client_(mongocxx::client(mongocxx::uri(_uri)))
    , dataset_dir_(_dataset_dir)
{
    auto database = client_["meteor"];
    auto cursor = database["SseSamples"].find({});
    for (auto&& doc : cursor) {
        image_annotations_.emplace_back(ImageAnnotation::create(doc, dataset_dir_));
    }
}

Polygon2Mask::~Polygon2Mask()
{
}

Polygon2Mask::Ptr Polygon2Mask::create(const std::string& _uri, const std::string& _dataset_dir)
{
    Polygon2Mask::Ptr obj_ptr;
    obj_ptr = std::make_shared<Polygon2Mask>(_uri, _dataset_dir);
    return obj_ptr;
}

} // namespace image_base

#endif