#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <iostream>
#include <vector>

const float CONF_THRESHOLD = 0.25f;
const float NMS_THRESHOLD  = 0.45f;
const int   INPUT_SIZE     = 640;

struct Detection {
    cv::Rect box;
    float    confidence;
};

std::vector<Detection> postProcess(
    const std::vector<cv::Mat>& outputs,
    int imgW, int imgH)
{
    std::vector<Detection> detections;
    std::vector<cv::Rect>  boxes;
    std::vector<float>     confidences;

    cv::Mat output = outputs[0];
    output = output.reshape(1, output.size[1]);
    cv::transpose(output, output);

    float xScale = (float)imgW / INPUT_SIZE;
    float yScale = (float)imgH / INPUT_SIZE;

    for (int i = 0; i < output.rows; i++) {
        float conf = output.at<float>(i, 4);
        if (conf < CONF_THRESHOLD) continue;

        float cx = output.at<float>(i, 0) * xScale;
        float cy = output.at<float>(i, 1) * yScale;
        float w  = output.at<float>(i, 2) * xScale;
        float h  = output.at<float>(i, 3) * yScale;

        int x1 = (int)(cx - w / 2);
        int y1 = (int)(cy - h / 2);

        boxes.push_back(cv::Rect(x1, y1, (int)w, (int)h));
        confidences.push_back(conf);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices) {
        Detection det;
        det.box        = boxes[idx];
        det.confidence = confidences[idx];
        detections.push_back(det);
    }

    return detections;
}

class FireDetectionNode : public rclcpp::Node
{
public:
    FireDetectionNode() : Node("fire_detection_node"), lastWarnTime_(this->get_clock()->now())
    {
        this->declare_parameter<std::string>("model_path",
            "/home/cho/lch_ws/src/pt_pkg/best.onnx");
        this->declare_parameter<std::string>("image_topic",
            "/image_raw/compressed");

        std::string modelPath  = this->get_parameter("model_path").as_string();
        std::string imageTopic = this->get_parameter("image_topic").as_string();

        net_ = cv::dnn::readNetFromONNX(modelPath);
        if (net_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "모델 로드 실패: %s", modelPath.c_str());
            rclcpp::shutdown();
            return;
        }

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        RCLCPP_INFO(this->get_logger(), "모델 로드 완료: %s", modelPath.c_str());

        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            imageTopic, 1,
            std::bind(&FireDetectionNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "구독 토픽: %s", imageTopic.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        if (frameCount_++ % 3 == 0) {
            int imgH = frame.rows;
            int imgW = frame.cols;

            cv::Mat blob = cv::dnn::blobFromImage(
                frame, 1.0 / 255.0,
                cv::Size(INPUT_SIZE, INPUT_SIZE),
                cv::Scalar(0, 0, 0), true, false);

            net_.setInput(blob);
            std::vector<cv::Mat> outputs;
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());

            lastDetections_ = postProcess(outputs, imgW, imgH);
        }

        for (const auto& det : lastDetections_) {
            cv::rectangle(frame, det.box, cv::Scalar(0, 0, 255), 2);

            std::string label = "Fire: " + std::to_string((int)(det.confidence * 100)) + "%";
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);

            int y = std::max(det.box.y, labelSize.height);
            cv::rectangle(frame,
                cv::Point(det.box.x, y - labelSize.height),
                cv::Point(det.box.x + labelSize.width, y + baseLine),
                cv::Scalar(0, 0, 255), cv::FILLED);

            cv::putText(frame, label,
                cv::Point(det.box.x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
        }

        bool fireDetected = !lastDetections_.empty();
        if (fireDetected) {
            auto now = this->get_clock()->now();
            if ((now - lastWarnTime_).seconds() >= 2.0) {
                RCLCPP_WARN(this->get_logger(), "!! FIRE DETECTED !!");
                lastWarnTime_ = now;
            }
            cv::putText(frame, "!! FIRE DETECTED !!",
                cv::Point(20, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1.2,
                cv::Scalar(0, 0, 255), 3);
        } else {
            cv::putText(frame, "No Fire",
                cv::Point(20, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("Fire Detection", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    cv::dnn::Net net_;
    int frameCount_ = 0;
    std::vector<Detection> lastDetections_;
    rclcpp::Time lastWarnTime_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireDetectionNode>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}