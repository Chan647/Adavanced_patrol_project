#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <map>

namespace fs = std::filesystem;

const float CONF_THRESHOLD = 0.60f;
const float NMS_THRESHOLD  = 0.45f;
const int   INPUT_SIZE     = 640;

const std::string BASE_PATH = "/home/cho/lch_ws/src/pt_pkg/src";

struct Detection {
    cv::Rect box;
    float    confidence;
};

struct FaceDetection {
    cv::Rect    box;
    std::string name;
    double      distance;
};

std::vector<Detection> postProcessFire(
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

        int x1 = std::max(0, (int)(cx - w / 2));
        int y1 = std::max(0, (int)(cy - h / 2));

        boxes.push_back(cv::Rect(x1, y1, (int)w, (int)h));
        confidences.push_back(conf);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences,
                      CONF_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices)
        detections.push_back({boxes[idx], confidences[idx]});

    return detections;
}

std::vector<Detection> postProcessPerson(
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
        float maxScore = 0;
        int   maxClass = 0;
        for (int c = 0; c < 80; c++) {
            float score = output.at<float>(i, 4 + c);
            if (score > maxScore) {
                maxScore = score;
                maxClass = c;
            }
        }

        if (maxClass != 0 || maxScore < CONF_THRESHOLD) continue;

        float cx = output.at<float>(i, 0) * xScale;
        float cy = output.at<float>(i, 1) * yScale;
        float w  = output.at<float>(i, 2) * xScale;
        float h  = output.at<float>(i, 3) * yScale;

        int x1 = std::max(0, (int)(cx - w / 2));
        int y1 = std::max(0, (int)(cy - h / 2));

        boxes.push_back(cv::Rect(x1, y1, (int)w, (int)h));
        confidences.push_back(maxScore);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences,
                      CONF_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices)
        detections.push_back({boxes[idx], confidences[idx]});

    return detections;
}

class FaceRecognizer {
public:
    FaceRecognizer() {
        face_detector_ = cv::dnn::readNetFromTensorflow(
            BASE_PATH + "/opencv_face_detector_uint8.pb",
            BASE_PATH + "/opencv_face_detector.pbtxt");

        if (face_detector_.empty()) {
            std::cerr << "얼굴 감지 모델 로드 실패!" << std::endl;
            return;
        }
        face_detector_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        face_detector_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        recognizer_ = cv::face::LBPHFaceRecognizer::create();

        std::string model_path = BASE_PATH + "/face_model.yml";
        std::string name_path  = BASE_PATH + "/name_map.txt";

        if (fs::exists(model_path)) {
            recognizer_->read(model_path);
            loadNameMap(name_path);
            std::cout << "얼굴 모델 로드 완료: " << model_path << std::endl;
        } else {
            std::cerr << "face_model.yml 없음 - register_face 먼저 실행하세요" << std::endl;
        }
    }

    std::vector<FaceDetection> detect(cv::Mat& frame) {
        std::vector<FaceDetection> results;
        if (frame.empty()) return results;

        int imgW = frame.cols;
        int imgH = frame.rows;

        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0,
            cv::Size(300, 300),
            cv::Scalar(104, 177, 123));

        face_detector_.setInput(blob);
        cv::Mat detections = face_detector_.forward();
        cv::Mat det_mat(detections.size[2], detections.size[3],
                        CV_32F, detections.ptr<float>());

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        for (int i = 0; i < det_mat.rows; i++) {
            float confidence = det_mat.at<float>(i, 2);
            if (confidence < 0.7f) continue;

            int x1 = std::max(0, (int)(det_mat.at<float>(i, 3) * imgW));
            int y1 = std::max(0, (int)(det_mat.at<float>(i, 4) * imgH));
            int x2 = std::min(imgW, (int)(det_mat.at<float>(i, 5) * imgW));
            int y2 = std::min(imgH, (int)(det_mat.at<float>(i, 6) * imgH));

            if (x2 - x1 <= 0 || y2 - y1 <= 0) continue;

            cv::Rect face_rect(x1, y1, x2 - x1, y2 - y1);
            cv::Mat face_roi = gray(face_rect);
            cv::resize(face_roi, face_roi, cv::Size(100, 100));

            std::string name = "침입자";
            double dist = 999.0;

            if (!id_to_name_.empty()) {
                int label;
                recognizer_->predict(face_roi, label, dist);
                name = (dist < unknown_threshold_) ?
                    id_to_name_[label] : "침입자";
            }

            results.push_back({face_rect, name, dist});
        }

        return results;
    }

    bool isIntruder(const std::string& name) {
        return name == "침입자";
    }

private:
    cv::dnn::Net face_detector_;
    cv::Ptr<cv::face::LBPHFaceRecognizer> recognizer_;
    std::map<int, std::string> id_to_name_;
    double unknown_threshold_ = 120.0;

    void loadNameMap(const std::string& path) {
        std::ifstream name_file(path);
        int id; std::string name;
        while (name_file >> id >> name) {
            id_to_name_[id] = name;
        }
        std::cout << "이름 매핑 로드 완료: "
                  << id_to_name_.size() << "명" << std::endl;
    }
};

class DetectionNode : public rclcpp::Node
{
public:
    DetectionNode()
        : Node("detection_node"),
          lastWarnTime_(this->get_clock()->now()),
          last_person_time_(rclcpp::Time(0))
    {
        this->declare_parameter<std::string>("fire_model_path",
            BASE_PATH + "/best.onnx");
        this->declare_parameter<std::string>("person_model_path",
            BASE_PATH + "/yolov8n.onnx");
        this->declare_parameter<std::string>("image_topic",
            "/image_raw/compressed");

        std::string fireModelPath   = this->get_parameter("fire_model_path").as_string();
        std::string personModelPath = this->get_parameter("person_model_path").as_string();
        std::string imageTopic      = this->get_parameter("image_topic").as_string();

        fireNet_ = cv::dnn::readNetFromONNX(fireModelPath);
        if (fireNet_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "화재 모델 로드 실패: %s", fireModelPath.c_str());
            rclcpp::shutdown(); return;
        }
        fireNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        fireNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        RCLCPP_INFO(this->get_logger(), "화재 모델 로드 완료");

        personNet_ = cv::dnn::readNetFromONNX(personModelPath);
        if (personNet_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "사람 모델 로드 실패: %s", personModelPath.c_str());
            rclcpp::shutdown(); return;
        }
        personNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        personNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        RCLCPP_INFO(this->get_logger(), "사람 모델 로드 완료");

        faceRecognizer_ = std::make_unique<FaceRecognizer>();
        RCLCPP_INFO(this->get_logger(), "얼굴 인식기 초기화 완료");

        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            imageTopic, rclcpp::SensorDataQoS(),
            std::bind(&DetectionNode::imageCallback,
                      this, std::placeholders::_1));

        person_pub_   = this->create_publisher<std_msgs::msg::Bool>("/person_detected", 10);
        bbox_pub_     = this->create_publisher<geometry_msgs::msg::Vector3>("/person_bbox", 10);
        intruder_pub_ = this->create_publisher<std_msgs::msg::Bool>("/intruder_detected", 10);
        face_pub_     = this->create_publisher<std_msgs::msg::String>("/face_result", 10);
        face_bbox_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/face_bbox", 10);

        RCLCPP_INFO(this->get_logger(),
            "구독 토픽: %s", imageTopic.c_str());
    }

private:
    void imageCallback(
        const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame = cv::imdecode(
            cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        int imgH = frame.rows;
        int imgW = frame.cols;

        if (frameCount_++ % 3 == 0) {
            cv::Mat blob = cv::dnn::blobFromImage(
                frame, 1.0 / 255.0,
                cv::Size(INPUT_SIZE, INPUT_SIZE),
                cv::Scalar(0, 0, 0), true, false);

            std::vector<cv::Mat> fireOutputs;
            fireNet_.setInput(blob);
            fireNet_.forward(fireOutputs,
                fireNet_.getUnconnectedOutLayersNames());
            lastFireDets_ = postProcessFire(fireOutputs, imgW, imgH);

            if (lastFireDets_.size() > 1) {
                auto max_it = std::max_element(
                    lastFireDets_.begin(), lastFireDets_.end(),
                    [](const Detection& a, const Detection& b) {
                        return a.box.area() < b.box.area();
                    });
                lastFireDets_ = {*max_it};
            }

            std::vector<cv::Mat> personOutputs;
            personNet_.setInput(blob);
            personNet_.forward(personOutputs,
                personNet_.getUnconnectedOutLayersNames());
            lastPersonDets_ = postProcessPerson(personOutputs, imgW, imgH);

            if (frameCount_ % 20 == 0) {
                if (target_locked_) {
                    cv::Mat target_roi = frame(target_box_ & cv::Rect(0, 0, imgW, imgH));
                    lastFaceDets_ = faceRecognizer_->detect(target_roi);
                } else {
                    lastFaceDets_ = faceRecognizer_->detect(frame);
                }
            }
        }
        for (const auto& det : lastFireDets_) {
            cv::rectangle(frame, det.box, cv::Scalar(0, 0, 255), 2);
            std::string label = "Fire: " +
                std::to_string((int)(det.confidence * 100)) + "%";
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label,
                cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
            int y = std::max(det.box.y, labelSize.height);
            cv::rectangle(frame,
                cv::Point(det.box.x, y - labelSize.height),
                cv::Point(det.box.x + labelSize.width, y + baseLine),
                cv::Scalar(0, 0, 255), cv::FILLED);
            cv::putText(frame, label, cv::Point(det.box.x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
        }

        for (const auto& det : lastPersonDets_) {
            cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
            std::string label = "Person: " +
                std::to_string((int)(det.confidence * 100)) + "%";
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label,
                cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
            int y = std::max(det.box.y, labelSize.height);
            cv::rectangle(frame,
                cv::Point(det.box.x, y - labelSize.height),
                cv::Point(det.box.x + labelSize.width, y + baseLine),
                cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(frame, label, cv::Point(det.box.x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 0), 2);
        }

        bool intruderDetected = false;
        std::string faceResultStr = "";

        for (const auto& det : lastFaceDets_) {
            bool isIntruder = faceRecognizer_->isIntruder(det.name);
            cv::Scalar color = isIntruder ?
                cv::Scalar(0, 0, 255) : cv::Scalar(255, 100, 0);

            cv::rectangle(frame, det.box, color, 2);

            std::string label = det.name + " (" +
                std::to_string((int)det.distance) + ")";
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label,
                cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
            int y = std::max(det.box.y, labelSize.height);
            cv::rectangle(frame,
                cv::Point(det.box.x, y - labelSize.height),
                cv::Point(det.box.x + labelSize.width, y + baseLine),
                color, cv::FILLED);
            cv::putText(frame, label, cv::Point(det.box.x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);

            if (isIntruder) intruderDetected = true;
            faceResultStr += det.name + " ";
        }

        bool fireDetected   = !lastFireDets_.empty();
        bool personDetected = !lastPersonDets_.empty();

        if (fireDetected) {
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

        if (personDetected) {
            cv::putText(frame,
                "Person: " + std::to_string((int)lastPersonDets_.size()),
                cv::Point(20, 100),
                cv::FONT_HERSHEY_SIMPLEX, 1.2,
                cv::Scalar(0, 255, 0), 3);
        } else {
            cv::putText(frame, "No Person",
                cv::Point(20, 100),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2);
        }

        if (intruderDetected) {
            cv::putText(frame, "!! 침입자 !!",
                cv::Point(20, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1.2,
                cv::Scalar(0, 0, 255), 3);
        } else if (!lastFaceDets_.empty()) {
            cv::putText(frame, "Face: OK",
                cv::Point(20, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(255, 100, 0), 2);
        }

        auto now = this->get_clock()->now();
        if ((now - lastWarnTime_).seconds() >= 2.0) {
            if (fireDetected)
                RCLCPP_WARN(this->get_logger(), "!! FIRE DETECTED !!");
            if (personDetected)
                RCLCPP_INFO(this->get_logger(),
                    "사람 감지: %d명", (int)lastPersonDets_.size());
            if (intruderDetected)
                RCLCPP_WARN(this->get_logger(), "!! 침입자 감지 !!");
            lastWarnTime_ = now;
        }

        if (personDetected && !lastPersonDets_.empty()) {
            if (!target_locked_) {
                auto& det = *std::max_element(
                    lastPersonDets_.begin(), lastPersonDets_.end(),
                    [](const Detection& a, const Detection& b) {
                        return a.box.area() < b.box.area();
                    });
                target_box_ = det.box;
                target_locked_ = true;
            } else {
                auto& det = *std::min_element(
                    lastPersonDets_.begin(), lastPersonDets_.end(),
                    [this](const Detection& a, const Detection& b) {
                        int ax = a.box.x + a.box.width  / 2;
                        int ay = a.box.y + a.box.height / 2;
                        int bx = b.box.x + b.box.width  / 2;
                        int by = b.box.y + b.box.height / 2;
                        int tx = target_box_.x + target_box_.width  / 2;
                        int ty = target_box_.y + target_box_.height / 2;
                        int da = (ax-tx)*(ax-tx) + (ay-ty)*(ay-ty);
                        int db = (bx-tx)*(bx-tx) + (by-ty)*(by-ty);
                        return da < db;
                    });
                target_box_ = det.box;
            }

            geometry_msgs::msg::Vector3 bbox_msg;
            bbox_msg.x = (target_box_.x + target_box_.width  / 2.0 - imgW / 2.0) / (imgW / 2.0);
            bbox_msg.y = (target_box_.y + target_box_.height / 2.0 - imgH / 2.0) / (imgH / 2.0);
            bbox_msg.z = (double)target_box_.area() / (imgW * imgH);
            last_bbox_z_ = bbox_msg.z;
            bbox_pub_->publish(bbox_msg);
        }

        bool realDetected = !lastPersonDets_.empty();
        if (realDetected) {
            last_person_time_ = this->get_clock()->now();
        }

        if (realDetected) {
            personDetected = true;
        } else {
            if (last_person_time_.nanoseconds() == 0) {
                target_locked_ = false;
                last_bbox_z_ = 0.0;
            } else {
                double elapsed = (this->get_clock()->now() - last_person_time_).seconds();
                if (elapsed < 0.5) {
                    personDetected = true;
                } else {
                    target_locked_ = false;
                    last_bbox_z_ = 0.0;
                }
            }
        }

        std_msgs::msg::Bool person_msg;
        person_msg.data = personDetected;
        person_pub_->publish(person_msg);

        std_msgs::msg::Bool intruder_msg;
        intruder_msg.data = intruderDetected;
        intruder_pub_->publish(intruder_msg);

        if (!faceResultStr.empty()) {
            std_msgs::msg::String face_msg;
            face_msg.data = faceResultStr;
            face_pub_->publish(face_msg);
        }

        if (!lastFaceDets_.empty()) {
            auto& face = lastFaceDets_[0];
            geometry_msgs::msg::Vector3 face_bbox_msg;

            if (target_locked_) {
                int abs_x = face.box.x + target_box_.x;
                int abs_y = face.box.y + target_box_.y;
                face_bbox_msg.x = (abs_x + face.box.width  / 2.0 - imgW / 2.0) / (imgW / 2.0);
                face_bbox_msg.y = (abs_y + face.box.height / 2.0 - imgH / 2.0) / (imgH / 2.0);
            } else {
                face_bbox_msg.x = (face.box.x + face.box.width  / 2.0 - imgW / 2.0) / (imgW / 2.0);
                face_bbox_msg.y = (face.box.y + face.box.height / 2.0 - imgH / 2.0) / (imgH / 2.0);
            }
            face_bbox_pub_->publish(face_bbox_msg);
        }

        if (target_locked_) {
            cv::rectangle(frame, target_box_, cv::Scalar(0, 255, 255), 3);
            cv::putText(frame, "TARGET",
                cv::Point(target_box_.x, target_box_.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2);
        }
        
        cv::imshow("Detection", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    cv::dnn::Net fireNet_;
    cv::dnn::Net personNet_;
    int frameCount_ = 0;
    std::vector<Detection> lastFireDets_;
    std::vector<Detection> lastPersonDets_;
    rclcpp::Time lastWarnTime_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr person_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bbox_pub_;
    std::unique_ptr<FaceRecognizer> faceRecognizer_;
    std::vector<FaceDetection> lastFaceDets_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intruder_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr face_bbox_pub_;
    rclcpp::Time last_person_time_;

    bool target_locked_{false};
    cv::Rect target_box_;
    double last_bbox_z_{0.0};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionNode>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}