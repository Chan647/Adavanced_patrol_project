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
#include <chrono> 
#include <sensor_msgs/msg/image.hpp>

namespace fs = std::filesystem;

const float CONF_THRESHOLD      = 0.60f; 
const float FIRE_CONF_THRESHOLD = 0.65f; 
const float NMS_THRESHOLD       = 0.45f;
const int   INPUT_SIZE          = 640;

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

std::vector<Detection> postProcessFire(const std::vector<cv::Mat>& outputs, int imgW, int imgH) {
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
        if (conf < FIRE_CONF_THRESHOLD) continue;

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
    cv::dnn::NMSBoxes(boxes, confidences, FIRE_CONF_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices)
        detections.push_back({boxes[idx], confidences[idx]});

    return detections;
}

std::vector<Detection> postProcessPerson(const std::vector<cv::Mat>& outputs, int imgW, int imgH) {
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
    cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, indices);

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

        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), cv::Scalar(104, 177, 123));
        face_detector_.setInput(blob);
        cv::Mat detections = face_detector_.forward();
        cv::Mat det_mat(detections.size[2], detections.size[3], CV_32F, detections.ptr<float>());

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

            std::string name = "Unknown";
            double dist = 999.0;

            if (!id_to_name_.empty()) {
                int label;
                recognizer_->predict(face_roi, label, dist);
                name = (dist < unknown_threshold_) ? id_to_name_[label] : "Unknown";
            }
            results.push_back({face_rect, name, dist});
        }
        return results;
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
        std::cout << "이름 매핑 로드 완료: " << id_to_name_.size() << "명" << std::endl;
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
        this->declare_parameter<std::string>("fire_model_path", BASE_PATH + "/best.onnx");
        this->declare_parameter<std::string>("person_model_path", BASE_PATH + "/yolov8n.onnx");
        this->declare_parameter<std::string>("image_topic", "/image_raw/compressed");

        std::string fireModelPath   = this->get_parameter("fire_model_path").as_string();
        std::string personModelPath = this->get_parameter("person_model_path").as_string();
        std::string imageTopic      = this->get_parameter("image_topic").as_string();

        fireNet_ = cv::dnn::readNetFromONNX(fireModelPath);
        if (fireNet_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "화재 모델 로드 실패: %s", fireModelPath.c_str());
            rclcpp::shutdown(); return;
        }
        fireNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        fireNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        personNet_ = cv::dnn::readNetFromONNX(personModelPath);
        if (personNet_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "사람 모델 로드 실패: %s", personModelPath.c_str());
            rclcpp::shutdown(); return;
        }
        personNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        personNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        faceRecognizer_ = std::make_unique<FaceRecognizer>();

        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            imageTopic, rclcpp::SensorDataQoS(),
            std::bind(&DetectionNode::imageCallback, this, std::placeholders::_1));

        person_pub_    = this->create_publisher<std_msgs::msg::Bool>("/person_detected", 10);
        bbox_pub_      = this->create_publisher<geometry_msgs::msg::Vector3>("/person_bbox", 10);
        intruder_pub_  = this->create_publisher<std_msgs::msg::Bool>("/intruder_detected", 10);
        face_pub_      = this->create_publisher<std_msgs::msg::String>("/face_result", 10);
        face_bbox_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/face_bbox", 10);
        fire_pub_      = this->create_publisher<std_msgs::msg::Bool>("/fire_detected", 10);
        

        annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detection/annotated_image", 10);

        // 1. 객체 감지 스위치 제어 구독
        fire_toggle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/toggle_fire_detection", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                is_fire_enabled_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "🔥 [웹 명령] 화재 감지: %s", is_fire_enabled_ ? "ON" : "OFF");
            });

        person_toggle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/toggle_person_detection", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                is_person_enabled_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "👤 [웹 명령] 사람 감지: %s", is_person_enabled_ ? "ON" : "OFF");
            });

        // 2. 심장박동 (Heartbeat) 1초마다 발사
        heartbeat_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cpp_node_heartbeat", 10);
        heartbeat_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            std_msgs::msg::Bool msg;
            msg.data = true;
            heartbeat_pub_->publish(msg);
        });

        RCLCPP_INFO(this->get_logger(), "✅ Detection Node 시작 (최적화 완료)");
    }

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        int imgH = frame.rows;
        int imgW = frame.cols;

        // 💡 객체 감지가 켜져 있을 때만 AI 연산 수행
        if (is_fire_enabled_ || is_person_enabled_) {
            
            // 🔥 CPU 과부하 방지 (10프레임당 1번만 YOLO 실행)
            if (frameCount_++ % 10 == 0) {
                cv::Mat blob = cv::dnn::blobFromImage(
                    frame, 1.0 / 255.0,
                    cv::Size(INPUT_SIZE, INPUT_SIZE),
                    cv::Scalar(0, 0, 0), true, false);

                // [선택 연산 1] 화재 감지가 ON일 때만 실행
                if (is_fire_enabled_) {
                    std::vector<cv::Mat> fireOutputs;
                    fireNet_.setInput(blob);
                    fireNet_.forward(fireOutputs, fireNet_.getUnconnectedOutLayersNames());
                    lastFireDets_ = postProcessFire(fireOutputs, imgW, imgH);

                    if (lastFireDets_.size() > 1) {
                        auto max_it = std::max_element(
                            lastFireDets_.begin(), lastFireDets_.end(),
                            [](const Detection& a, const Detection& b) { return a.box.area() < b.box.area(); });
                        lastFireDets_ = {*max_it};
                    }
                } else {
                    lastFireDets_.clear();
                }

                // [선택 연산 2] 사람 감지가 ON일 때만 실행
                if (is_person_enabled_) {
                    std::vector<cv::Mat> personOutputs;
                    personNet_.setInput(blob);
                    personNet_.forward(personOutputs, personNet_.getUnconnectedOutLayersNames());
                    lastPersonDets_ = postProcessPerson(personOutputs, imgW, imgH);
                } else {
                    lastPersonDets_.clear();
                }
            }


            bool realDetected = !lastPersonDets_.empty();
            if (!realDetected) {
                lastFaceDets_.clear();
                faceFrameCount_ = 0;
            } else {
                // 🔥 얼굴 인식은 무거우므로 20프레임당 1번만 실행
                if (faceFrameCount_++ % 20 == 0) {
                    if (target_locked_) {
                        int marginX        = 40;
                        int marginY_top    = 60;
                        int marginY_bottom = 20;

                        cv::Rect expanded = target_box_;
                        expanded.x      = std::max(0, expanded.x - marginX);
                        expanded.y      = std::max(0, expanded.y - marginY_top);
                        expanded.width  = std::min(imgW - expanded.x, expanded.width  + 2 * marginX);
                        expanded.height = std::min(imgH - expanded.y, expanded.height + marginY_top + marginY_bottom);

                        cv::Mat target_roi = frame(expanded);
                        lastFaceDets_ = faceRecognizer_->detect(target_roi);

                        for (auto& face : lastFaceDets_) {
                            face.box.x += expanded.x;
                            face.box.y += expanded.y;
                        }
                    } else {
                        lastFaceDets_ = faceRecognizer_->detect(frame);
                    }
                }
            }

            // ==========================================================
            // 💡 [수정] 스위치 상태에 따라 개별적으로 박스와 텍스트 출력
            // ==========================================================
            std::string faceResultStr = "";
            bool faceIdentified = false;
            bool fireDetected   = !lastFireDets_.empty();
            bool personDetected = !lastPersonDets_.empty();

            // 🔥 [화재 BBox & 텍스트] 화재 감지가 ON일 때만 표시
            if (is_fire_enabled_) {
                for (const auto& det : lastFireDets_) {
                    cv::rectangle(frame, det.box, cv::Scalar(0, 0, 255), 2);
                    std::string label = "Fire: " + std::to_string((int)(det.confidence * 100)) + "%";
                    int baseLine;
                    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
                    int y = std::max(det.box.y, labelSize.height);
                    cv::rectangle(frame, cv::Point(det.box.x, y - labelSize.height),
                                  cv::Point(det.box.x + labelSize.width, y + baseLine),
                                  cv::Scalar(0, 0, 255), cv::FILLED);
                    cv::putText(frame, label, cv::Point(det.box.x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                }

                if (fireDetected) {
                    cv::putText(frame, "!! FIRE DETECTED !!", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
                } else {
                    cv::putText(frame, "Fire: ON ", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }
            }

            // 👤 [사람/얼굴 BBox & 텍스트] 사람 감지가 ON일 때만 표시
            if (is_person_enabled_) {
                for (const auto& det : lastPersonDets_) {
                    cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
                    std::string label = "Person: " + std::to_string((int)(det.confidence * 100)) + "%";
                    int baseLine;
                    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
                    int y = std::max(det.box.y, labelSize.height);
                    cv::rectangle(frame, cv::Point(det.box.x, y - labelSize.height),
                                  cv::Point(det.box.x + labelSize.width, y + baseLine),
                                  cv::Scalar(0, 255, 0), cv::FILLED);
                    cv::putText(frame, label, cv::Point(det.box.x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
                }

                for (const auto& det : lastFaceDets_) {
                    if (det.name == "Unknown") continue;

                    cv::rectangle(frame, det.box, cv::Scalar(255, 100, 0), 2);
                    std::string label = det.name + " (" + std::to_string((int)det.distance) + ")";
                    int baseLine;
                    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
                    int y = std::max(det.box.y, labelSize.height);
                    cv::rectangle(frame, cv::Point(det.box.x, y - labelSize.height),
                                  cv::Point(det.box.x + labelSize.width, y + baseLine),
                                  cv::Scalar(255, 100, 0), cv::FILLED);
                    cv::putText(frame, label, cv::Point(det.box.x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                    faceResultStr += det.name + " ";
                    faceIdentified = true;
                }

                if (personDetected) {
                    cv::putText(frame, "Person: " + std::to_string((int)lastPersonDets_.size()), cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
                } else {
                    cv::putText(frame, "Person: ON ", cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }

                if (faceIdentified) {
                    cv::putText(frame, "Face: OK", cv::Point(20, 130), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 100, 0), 2);
                }
            }
            // ==========================================================

            auto now = this->get_clock()->now();
            if ((now - lastWarnTime_).seconds() >= 2.0) {
                if (fireDetected) RCLCPP_WARN(this->get_logger(), "!! FIRE DETECTED !!");
                if (personDetected) RCLCPP_INFO(this->get_logger(), "사람 감지: %d명", (int)lastPersonDets_.size());
                lastWarnTime_ = now;
            }

            if (personDetected && !lastPersonDets_.empty()) {
                if (!target_locked_) {
                    auto& det = *std::max_element(lastPersonDets_.begin(), lastPersonDets_.end(),
                        [](const Detection& a, const Detection& b) { return a.box.area() < b.box.area(); });
                    target_box_    = det.box;
                    target_locked_ = true;
                } else {
                    auto& det = *std::min_element(lastPersonDets_.begin(), lastPersonDets_.end(),
                        [this](const Detection& a, const Detection& b) {
                            int ax = a.box.x + a.box.width  / 2, ay = a.box.y + a.box.height / 2;
                            int bx = b.box.x + b.box.width  / 2, by = b.box.y + b.box.height / 2;
                            int tx = target_box_.x + target_box_.width / 2, ty = target_box_.y + target_box_.height / 2;
                            int da = (ax-tx)*(ax-tx) + (ay-ty)*(ay-ty);
                            int db = (bx-tx)*(bx-tx) + (by-ty)*(by-ty);
                            return da < db;
                        });
                    
                    int tx = target_box_.x + target_box_.width / 2, ty = target_box_.y + target_box_.height / 2;
                    int cx = det.box.x + det.box.width / 2, cy = det.box.y + det.box.height / 2;
                    int dist = (cx-tx)*(cx-tx) + (cy-ty)*(cy-ty);
                    
                    if (dist < 200*200) { target_box_ = det.box; } 
                }

                geometry_msgs::msg::Vector3 bbox_msg;
                bbox_msg.x = (target_box_.x + target_box_.width  / 2.0 - imgW / 2.0) / (imgW / 2.0);
                bbox_msg.y = (target_box_.y + target_box_.height / 2.0 - imgH / 2.0) / (imgH / 2.0);
                bbox_msg.z = (double)target_box_.area() / (imgW * imgH);
                last_bbox_z_ = bbox_msg.z;
                bbox_pub_->publish(bbox_msg);
            }

            if (realDetected) {
                last_person_time_ = this->get_clock()->now();
            }

            if (realDetected) {
                personDetected = true;
            } else {
                if (last_person_time_.nanoseconds() == 0) {
                    target_locked_ = false;
                    last_bbox_z_   = 0.0;
                } else {
                    double elapsed = (this->get_clock()->now() - last_person_time_).seconds();
                    if (elapsed < 0.5) {
                        personDetected = true;
                    } else {
                        target_locked_ = false;
                        last_bbox_z_   = 0.0;
                    }
                }
            }

            std_msgs::msg::Bool person_msg;
            person_msg.data = personDetected;
            person_pub_->publish(person_msg);

            std_msgs::msg::Bool intruder_msg;
            intruder_msg.data = (personDetected && faceResultStr.find("Unknown") != std::string::npos);
            intruder_pub_->publish(intruder_msg);

            std_msgs::msg::Bool fire_out_msg;
            fire_out_msg.data = fireDetected;
            fire_pub_->publish(fire_out_msg);

            if (!faceResultStr.empty()) {
                std_msgs::msg::String face_msg;
                face_msg.data = faceResultStr;
                face_pub_->publish(face_msg);
            }

            if (faceIdentified && !lastFaceDets_.empty()) {
                for (const auto& face : lastFaceDets_) {
                    if (face.name == "Unknown") continue;
                    geometry_msgs::msg::Vector3 face_bbox_msg;
                    face_bbox_msg.x = (face.box.x + face.box.width  / 2.0 - imgW / 2.0) / (imgW / 2.0);
                    face_bbox_msg.y = (face.box.y + face.box.height / 2.0 - imgH / 2.0) / (imgH / 2.0);
                    face_bbox_pub_->publish(face_bbox_msg);
                    break;
                }
            }

            if (target_locked_) {
                cv::rectangle(frame, target_box_, cv::Scalar(0, 255, 255), 3);
                cv::putText(frame, "TARGET", cv::Point(target_box_.x, target_box_.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            }
        } 
        else {
            // 💡 객체 감지가 OFF일 때: 웹캠은 끊기지 않고 계속 송출하되, 감지 신호만 False로 보냄
            std_msgs::msg::Bool p_msg; p_msg.data = false;
            person_pub_->publish(p_msg);
            fire_pub_->publish(p_msg); 
            cv::putText(frame, "DETECTION: OFF", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
        }

        // 💡 웹 송출을 위해 이미지 리사이즈 후 Publish (ON/OFF 상관없이 항상 실행됨)
        cv::Mat web_frame;
        cv::resize(frame, web_frame, cv::Size(320, 240));

        sensor_msgs::msg::Image img_msg;
        img_msg.header.stamp = this->get_clock()->now();
        img_msg.header.frame_id = "camera_link";
        img_msg.height = web_frame.rows;
        img_msg.width = web_frame.cols;
        img_msg.encoding = "bgr8";
        img_msg.is_bigendian = false;
        img_msg.step = web_frame.step;
        size_t size = web_frame.step * web_frame.rows;
        img_msg.data.resize(size);
        memcpy(&img_msg.data[0], web_frame.data, size);
        
        annotated_pub_->publish(img_msg);
    }

    // ======================================================================
    // 💡 [수정 포인트] 시작 시 객체 감지 기본 상태 설정
    // ======================================================================
    // true  : 노드 실행 시 자동으로 AI 감지가 시작됩니다.
    // false : 노드 실행 시 대기하다가, 웹 패널에서 버튼을 눌러야 시작됩니다.
    bool is_fire_enabled_{false};   // 화재 감지 기본 상태
    bool is_person_enabled_{false}; // 사람 감지 기본 상태
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_toggle_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr person_toggle_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

    cv::dnn::Net fireNet_;
    cv::dnn::Net personNet_;

    int frameCount_     = 0;
    int faceFrameCount_ = 0;

    std::vector<Detection>     lastFireDets_;
    std::vector<Detection>     lastPersonDets_;
    std::vector<FaceDetection> lastFaceDets_;

    rclcpp::Time lastWarnTime_;
    rclcpp::Time last_person_time_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         person_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         intruder_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       face_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr face_bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         fire_pub_;

    std::unique_ptr<FaceRecognizer> faceRecognizer_;

    bool     target_locked_{false};
    cv::Rect target_box_;
    double   last_bbox_z_{0.0};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionNode>());
    rclcpp::shutdown();
    return 0;
}