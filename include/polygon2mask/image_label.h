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
    ImageAnnotation(const bsoncxx::document::view& documents_, const std::string& _dataset_dir);
    virtual ~ImageAnnotation();
    static ImageAnnotation::Ptr create(const bsoncxx::document::view& documents_, const std::string& _dataset_dir);

private:
    void createAnnotation();
    void createMask();
    void writeMaskImage();

    std::string dataset_dir_;
    std::string folder_;
    std::string filename_;
    std::string im_path_;
    std::string mask_path_;
    std::string soc_name_;
    std::vector<Annotation> annotations_;
    cv::Mat color_img_;
    cv::Mat mask_img_;
    bsoncxx::document::view documents_;
};

ImageAnnotation::ImageAnnotation(const bsoncxx::document::view& _doc, const std::string& _dataset_dir)
    : documents_(_doc)
{
    dataset_dir_ = _dataset_dir;
    folder_ = documents_["folder"].get_string().value.to_string();
    filename_ = documents_["file"].get_string().value.to_string();
    im_path_ = dataset_dir_ + folder_ + "/" + filename_;
    std::string mask_folder = folder_;
    mask_folder.erase(mask_folder.end() - 6, mask_folder.end());
    mask_folder += "masks/";
    mask_path_ = dataset_dir_ + mask_folder + filename_;

    createAnnotation();
    createMask();
    writeMaskImage();
}

ImageAnnotation::~ImageAnnotation()
{
}

ImageAnnotation::Ptr ImageAnnotation::create(const bsoncxx::document::view& documents_, const std::string& _dataset_dir)
{
    ImageAnnotation::Ptr obj_ptr;
    obj_ptr = std::allocate_shared<ImageAnnotation>(Eigen::aligned_allocator<ImageAnnotation>(), documents_, _dataset_dir);
    return obj_ptr;
}

void ImageAnnotation::createAnnotation()
{
    if (documents_["objects"] && documents_["objects"].type() == bsoncxx::type::k_array) {
        bsoncxx::array::view objects { documents_["objects"].get_array().value };
        for (const auto& object : objects) {
            /*create an annotation*/
            Annotation annotation;
            Polygon polygon;
            for (const auto& entity : object["polygon"].get_array().value) {
                polygon.emplace_back(cv::Point2d(entity["x"].get_double().value, entity["y"].get_double().value));
            }
            annotation = { object["classIndex"].get_int32().value, polygon };
            annotations_.emplace_back(annotation);
        }
    }
}

void ImageAnnotation::createMask()
{
    color_img_ = cv::imread(im_path_);
    mask_img_ = cv::Mat(color_img_.rows, color_img_.cols, CV_8UC1);
    mask_img_.setTo(cv::Scalar::all(0));
    for (const auto& annotation : annotations_) {
        const cv::Point* pt_ptr[1] = { annotation.second.data() };
        int npt[] = { (int)annotation.second.size() };
        cv::polylines(color_img_, pt_ptr, npt, 1, 1, cv::Scalar(255, 0, 0), 10, 8, 0);
        cv::fillPoly(mask_img_, pt_ptr, npt, 1, cv::Scalar(255, 255, 255));
    }

    // cv::resize(color_img_, color_img_, cv::Size(640, 480));
    // cv::resize(mask_img_, mask_img_, cv::Size(640, 480));

    // cv::imshow("polyline", color_img_);
    // cv::imshow("polyfill", mask_img_);
    // cv::waitKey(0);
}

void ImageAnnotation::writeMaskImage()
{
    bool write_check = cv::imwrite(mask_path_, mask_img_);
    if (!write_check)
        std::cout << "imwrite failed directory: " << mask_path_ << std::endl;
}

} // namespace image_base

#endif