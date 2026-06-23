#pragma once

#include "yolo_utilities.hpp"


namespace YOLO_DETECTOR {

/**
 * @brief YOLO12Detector class handles loading the YOLO model, preprocessing images, running inference, and postprocessing results.
 */
class YOLO12Detector {

public:

    /**
     * @brief Constructor to initialize the YOLO detector with model and label paths.
     * 
     * @param modelPath Path to the ONNX model file.
     * @param labelsPath Path to the file containing class labels.
     * @param useGPU Whether to use GPU for inference (default is false).
     */
    YOLO12Detector(const std::string &modelPath, const std::string &labelsPath, bool useGPU = false);
    
    /**
     * @brief Runs detection on the provided image.
     * 
     * @param image Input image for detection.
     * @param confThreshold Confidence threshold to filter detections (default is 0.5).
     * @param iouThreshold IoU threshold for Non-Maximum Suppression (default is 0.5).
     * @return std::vector<Detection> Vector of detections.
     */
    std::vector<Detection> detect(const cv::Mat &image, float confThreshold = 0.5f, float iouThreshold = 0.5f);
    
    /**
     * @brief Draws bounding boxes on the image based on detections.
     * 
     * @param image Image on which to draw.
     * @param detections Vector of detections.
     */
    void drawBoundingBox(cv::Mat &image, const std::vector<Detection> &detections) const {
        YOLO_DETECTOR::drawBoundingBox(image, detections, classNames, classColors);
    }
        
    /**
     * @brief Draws bounding boxes and semi-transparent masks on the image based on detections.
     * 
     * @param image Image on which to draw.
     * @param detections Vector of detections.
     * @param maskAlpha Alpha value for mask transparency (default is 0.5).
     */
    void drawBoundingBoxMask(cv::Mat &image, const std::vector<Detection> &detections, float maskAlpha = 0.5f) const {
        YOLO_DETECTOR::drawBoundingBoxMask(image, detections, classNames, classColors, maskAlpha);
    }

private:

    Ort::Env env{nullptr};                         // ONNX Runtime environment
    Ort::SessionOptions sessionOptions{nullptr};   // Session options for ONNX Runtime
    Ort::Session session{nullptr};                 // ONNX Runtime session for running inference
    bool isDynamicInputShape{};                    // Flag indicating if input shape is dynamic
    cv::Size inputImageShape;                      // Expected input image shape for the model

    // Vectors to hold allocated input and output node names
    std::vector<Ort::AllocatedStringPtr> inputNodeNameAllocatedStrings;
    std::vector<const char *> inputNames;
    std::vector<Ort::AllocatedStringPtr> outputNodeNameAllocatedStrings;
    std::vector<const char *> outputNames;

    size_t numInputNodes, numOutputNodes;          // Number of input and output nodes in the model

    std::vector<std::string> classNames;            // Vector of class names loaded from file
    std::vector<cv::Scalar> classColors;            // Vector of colors for each class

    /**
     * @brief Preprocesses the input image for model inference.
     * 
     * @param image Input image.
     * @param blob Reference to pointer where preprocessed data will be stored.
     * @param inputTensorShape Reference to vector representing input tensor shape.
     * @return cv::Mat Resized image after preprocessing.
     */
    cv::Mat preprocess(const cv::Mat &image, float *&blob, std::vector<int64_t> &inputTensorShape);
        
    /**
     * @brief Postprocesses the model output to extract detections.
     * 
     * @param originalImageSize Size of the original input image.
     * @param resizedImageShape Size of the image after preprocessing.
     * @param outputTensors Vector of output tensors from the model.
     * @param confThreshold Confidence threshold to filter detections.
     * @param iouThreshold IoU threshold for Non-Maximum Suppression.
     * @return std::vector<Detection> Vector of detections.
     */
    std::vector<Detection> postprocess(const cv::Size &originalImageSize, const cv::Size &resizedImageShape,
                                       const std::vector<Ort::Value> &outputTensors,
                                       float confThreshold, float iouThreshold);
    
};

} // namespace YOLO_DETECTOR
