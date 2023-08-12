#ifndef IMAGE_LABAL_H
#define IMAGE_LABAL_H

#include <Eigen/Eigen>
#include <bsoncxx/v_noabi/bsoncxx/document/element.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace image_base {
class ImageAnnotation {
public:
    using Ptr = std::shared_ptr<ImageAnnotation>;
    using Polygon = std::vector<cv::Point>;
    using Annotation = std::pair<int, Polygon>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageAnnotation(const bsoncxx::document::view& _doc, const std::string& _dataset_dir);
    virtual ~ImageAnnotation();
    static ImageAnnotation::Ptr create(const bsoncxx::document::view& _doc, const std::string& _dataset_dir);

private:
    void createAnnotation(const bsoncxx::document::view& _doc);
    void createMask();

    std::string dataset_dir_;
    std::string folder_;
    std::string filename_;
    std::string soc_name_;
    std::vector<Annotation> annotations_;
    cv::Mat color_img_;
    cv::Mat mask_img_;
};

ImageAnnotation::ImageAnnotation(const bsoncxx::document::view& _doc, const std::string& _dataset_dir)
{
    dataset_dir_ = _dataset_dir;
    folder_ = _doc["folder"].get_string().value.to_string();
    filename_ = _doc["file"].get_string().value.to_string();
    std::cout << "filename_: " << filename_ << std::endl;
    createAnnotation(_doc);
    if (annotations_.size() > 1)
        createMask();
    // std::cout << "annotations_[size]: " << annotations_.size() << std::endl;
}

ImageAnnotation::~ImageAnnotation()
{
}

ImageAnnotation::Ptr ImageAnnotation::create(const bsoncxx::document::view& _doc, const std::string& _dataset_dir)
{
    ImageAnnotation::Ptr obj_ptr;
    obj_ptr = std::allocate_shared<ImageAnnotation>(Eigen::aligned_allocator<ImageAnnotation>(), _doc, _dataset_dir);
    return obj_ptr;
}

void ImageAnnotation::createAnnotation(const bsoncxx::document::view& _doc)
{
    if (_doc["objects"] && _doc["objects"].type() == bsoncxx::type::k_array) {
        bsoncxx::array::view objects { _doc["objects"].get_array().value };
        int obj_idx = 0;
        for (const auto& object : objects) {
            /*create an annotation*/
            Annotation annotation;
            Polygon polygon;
            for (const auto& entity : object["polygon"].get_array().value) {
                polygon.emplace_back(cv::Point2d(entity["x"].get_double().value, entity["y"].get_double().value));
            }
            annotation = { object["classIndex"].get_int32().value, polygon };
            annotations_.emplace_back(annotation);
            std::cout << "[label,polygon_size]: "
                      << "[" << annotation.first << ", " << annotation.second.size() << "]" << std::endl;
            obj_idx++;
        }
    }
}

void ImageAnnotation::createMask()
{
    const std::string im_path = dataset_dir_ + folder_ + "/" + filename_;
    std::cout << "im_path: " << im_path << std::endl;
    color_img_ = cv::imread(im_path);
    mask_img_ = cv::Mat(color_img_.rows, color_img_.cols, CV_8UC1);
    mask_img_.setTo(cv::Scalar::all(0));
    for (const auto& annotation : annotations_) {
        const cv::Point* pt_ptr[1] = { annotation.second.data() };
        int npt[] = { (int)annotation.second.size() };
        cv::polylines(color_img_, pt_ptr, npt, 1, 1, cv::Scalar(255, 0, 0), 10, 8, 0);
        cv::fillPoly(mask_img_, pt_ptr, npt, 1, cv::Scalar(255, 255, 255));
    }
    cv::resize(color_img_, color_img_, cv::Size(640, 480));
    cv::resize(mask_img_, mask_img_, cv::Size(640, 480));
    cv::imshow("polyline", color_img_);
    cv::imshow("polyfill", mask_img_);
    cv::waitKey(0);
}

} // namespace image_base

#endif