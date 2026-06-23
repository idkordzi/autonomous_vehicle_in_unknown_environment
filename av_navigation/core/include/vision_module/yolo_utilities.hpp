#pragma once

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <unordered_map>
#include <thread>


namespace YOLO_DETECTOR {

constexpr float CONFIDENCE_THRESHOLD = 0.5f;  // confidence threshold for filtering detections
constexpr float IOU_THRESHOLD = 0.5f;         // IoU threshold for filtering detections

/**
 * @brief Struct to represent a bounding box
 */
struct BoundingBox {
    int x;
    int y;
    int width;
    int height;

    BoundingBox() : x(0), y(0), width(0), height(0) {}
    BoundingBox(int x_, int y_, int width_, int height_) : x(x_), y(y_), width(width_), height(height_) {}
};

/**
 * @brief Struct to represent a detection
 */
struct Detection {
    BoundingBox box;
    float conf  = 0.0f;
    int classId = 0;
};

/**
 * @brief A robust implementation of a clamp function.
 *        Restricts a value to lie within a specified range [low, high].
 *
 * @tparam T The type of the value to clamp. Should be an arithmetic type (int, float, etc.).
 * @param value The value to clamp.
 * @param low The lower bound of the range.
 * @param high The upper bound of the range.
 * @return const T& The clamped value, constrained to the range [low, high].
 *
 * @note If low > high, the function swaps the bounds automatically to ensure valid behavior.
 */
template <typename T>
typename std::enable_if<std::is_arithmetic<T>::value, T>::type
inline clamp(const T &value, const T &low, const T &high)
{
    // Ensure the range [low, high] is valid; swap if necessary
    T validLow = low < high ? low : high;
    T validHigh = low < high ? high : low;

    // Clamp the value to the range [validLow, validHigh]
    if (value < validLow)
        return validLow;
    if (value > validHigh)
        return validHigh;
    return value;
}

/**
 * @brief Loads class names from a given file path.
 * 
 * @param path Path to the file containing class names.
 * @return std::vector<std::string> Vector of class names.
 */
std::vector<std::string> getClassNames(const std::string &path);

/**
 * @brief Computes the product of elements in a vector.
 * 
 * @param vector Vector of integers.
 * @return size_t Product of all elements.
 */
size_t vectorProduct(const std::vector<int64_t> &vector);


/**
 * @brief Resizes an image with letterboxing to maintain aspect ratio.
 * 
 * @param image Input image.
 * @param outImage Output resized and padded image.
 * @param newShape Desired output size.
 * @param color Padding color (default is gray).
 * @param auto_ Automatically adjust padding to be multiple of stride.
 * @param scaleFill Whether to scale to fill the new shape without keeping aspect ratio.
 * @param scaleUp Whether to allow scaling up of the image.
 * @param stride Stride size for padding alignment.
 */
void letterBox(const cv::Mat& image, cv::Mat& outImage, const cv::Size& newShape,
               const cv::Scalar& color = cv::Scalar(114, 114, 114), bool auto_ = true,
               bool scaleFill = false, bool scaleUp = true, int stride = 32);

/**
 * @brief Scales detection coordinates back to the original image size.
 * 
 * @param imageShape Shape of the resized image used for inference.
 * @param bbox Detection bounding box to be scaled.
 * @param imageOriginalShape Original image size before resizing.
 * @param p_Clip Whether to clip the coordinates to the image boundaries.
 * @return BoundingBox Scaled bounding box.
 */
BoundingBox scaleCoords(const cv::Size &imageShape, BoundingBox coords, const cv::Size &imageOriginalShape, bool p_Clip);

/**
 * @brief Performs Non-Maximum Suppression (NMS) on the bounding boxes.
 * 
 * @param boundingBoxes Vector of bounding boxes.
 * @param scores Vector of confidence scores corresponding to each bounding box.
 * @param scoreThreshold Confidence threshold to filter boxes.
 * @param nmsThreshold IoU threshold for NMS.
 * @param indices Output vector of indices that survive NMS.
 */
void NMSBoxes(const std::vector<BoundingBox>& boundingBoxes, const std::vector<float>& scores,
              float scoreThreshold, float nmsThreshold, std::vector<int>& indices);


/**
 * @brief Generates a vector of colors for each class name.
 * 
 * @param classNames Vector of class names.
 * @param seed Seed for random color generation to ensure reproducibility.
 * @return std::vector<cv::Scalar> Vector of colors.
 */
std::vector<cv::Scalar> generateColors(const std::vector<std::string> &classNames, int seed = 42);

/**
 * @brief Draws bounding boxes and labels on the image based on detections.
 * 
 * @param image Image on which to draw.
 * @param detections Vector of detections.
 * @param classNames Vector of class names corresponding to object IDs.
 * @param colors Vector of colors for each class.
 */
void drawBoundingBox(cv::Mat &image, const std::vector<Detection> &detections,
                     const std::vector<std::string> &classNames, const std::vector<cv::Scalar> &colors);

/**
 * @brief Draws bounding boxes and semi-transparent masks on the image based on detections.
 * 
 * @param image Image on which to draw.
 * @param detections Vector of detections.
 * @param classNames Vector of class names corresponding to object IDs.
 * @param classColors Vector of colors for each class.
 * @param maskAlpha Alpha value for the mask transparency.
 */
void drawBoundingBoxMask(cv::Mat &image, const std::vector<Detection> &detections, const std::vector<std::string> &classNames,
                         const std::vector<cv::Scalar> &classColors, float maskAlpha = 0.4f);

}  // namespace YOLO_DETECTOR
