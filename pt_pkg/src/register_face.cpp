#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <map>
#include <vector>
#include <thread>
#include <mutex>

namespace fs = std::filesystem;

const std::string BASE_PATH = "/home/cho/lch_ws/src/pt_pkg/src";

// =============================================
// 카메라 구독 노드
// =============================================
class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", 1,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_frame_ = cv::imdecode(
                    cv::Mat(msg->data), cv::IMREAD_COLOR);
            });
        RCLCPP_INFO(this->get_logger(), "카메라 토픽 구독 시작");
    }

    cv::Mat getFrame() {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_frame_.clone();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    cv::Mat latest_frame_;
    std::mutex mutex_;
};

cv::dnn::Net face_detector;
std::shared_ptr<CameraNode> camera_node;

bool initDetector() {
    std::string pb_path    = BASE_PATH + "/opencv_face_detector_uint8.pb";
    std::string pbtxt_path = BASE_PATH + "/opencv_face_detector.pbtxt";

    face_detector = cv::dnn::readNetFromTensorflow(pb_path, pbtxt_path);

    if (face_detector.empty()) {
        std::cerr << "얼굴 감지 모델 로드 실패!" << std::endl;
        std::cerr << "경로 확인: " << pb_path << std::endl;
        return false;
    }
    std::cout << "얼굴 감지기 초기화 완료" << std::endl;
    return true;
}

cv::Mat detectAndCrop(cv::Mat& frame) {
    if (frame.empty()) return cv::Mat();

    int imgW = frame.cols;
    int imgH = frame.rows;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0,
        cv::Size(300, 300),
        cv::Scalar(104, 177, 123));

    face_detector.setInput(blob);
    cv::Mat detections = face_detector.forward();
    cv::Mat det_mat(detections.size[2], detections.size[3],
                    CV_32F, detections.ptr<float>());

    float best_conf = 0;
    cv::Rect best_rect;

    for (int i = 0; i < det_mat.rows; i++) {
        float conf = det_mat.at<float>(i, 2);
        if (conf < 0.7f) continue;

        int x1 = std::max(0, (int)(det_mat.at<float>(i, 3) * imgW));
        int y1 = std::max(0, (int)(det_mat.at<float>(i, 4) * imgH));
        int x2 = std::min(imgW, (int)(det_mat.at<float>(i, 5) * imgW));
        int y2 = std::min(imgH, (int)(det_mat.at<float>(i, 6) * imgH));

        if (x2 - x1 <= 0 || y2 - y1 <= 0) continue;

        if (conf > best_conf) {
            best_conf = conf;
            best_rect = cv::Rect(x1, y1, x2 - x1, y2 - y1);
        }
    }

    if (best_conf == 0) return cv::Mat();

    cv::rectangle(frame, best_rect, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, "Face Detected!",
        cv::Point(best_rect.x, best_rect.y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.7,
        cv::Scalar(0, 255, 0), 2);

    cv::Mat gray;
    cv::cvtColor(frame(best_rect), gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, gray, cv::Size(100, 100));
    return gray;
}

void registerFace() {
    std::string name;
    std::cout << "\n등록할 사람 이름 입력: ";
    std::cin >> name;

    std::string save_dir = BASE_PATH + "/faces/" + name;
    fs::create_directories(save_dir);

    int existing = 0;
    for (auto& f : fs::directory_iterator(save_dir)) {
        if (f.path().extension() == ".jpg") existing++;
    }

    int required = 10;
    int count = 0;

    std::cout << "\n=============================" << std::endl;
    std::cout << " [등록 모드] " << name << std::endl;
    std::cout << " SPACE : 촬영" << std::endl;
    std::cout << " Q     : 등록 완료" << std::endl;
    std::cout << " 목표  : " << required << "장" << std::endl;
    std::cout << " 다양한 각도로 촬영하세요!" << std::endl;
    std::cout << "=============================" << std::endl;

    while (rclcpp::ok()) {
        cv::Mat frame = camera_node->getFrame();

        if (frame.empty()) {
            std::cout << "카메라 토픽 대기 중..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        cv::Mat face_roi = detectAndCrop(frame);

        std::string status = "촬영: " + std::to_string(count) +
                             "/" + std::to_string(required);
        cv::putText(frame, status,
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.9,
            cv::Scalar(0, 255, 255), 2);

        if (face_roi.empty()) {
            cv::putText(frame, "얼굴을 카메라에 맞춰주세요",
                cv::Point(10, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 0, 255), 2);
        } else {
            cv::putText(frame, "SPACE: 촬영",
                cv::Point(10, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 0), 2);
        }

        cv::imshow("얼굴 등록 - " + name, frame);
        int key = cv::waitKey(1);

        if (key == 32) {
            if (face_roi.empty()) {
                std::cout << "얼굴이 감지되지 않았습니다. 다시 시도하세요." << std::endl;
                continue;
            }
            std::string filepath = save_dir + "/" +
                std::to_string(existing + count) + ".jpg";
            cv::imwrite(filepath, face_roi);
            count++;
            std::cout << count << "번째 촬영 완료! (" << filepath << ")" << std::endl;

            if (count >= required) {
                std::cout << "목표 달성! 등록 완료." << std::endl;
                break;
            }
        }
        else if (key == 'q' || key == 'Q') {
            if (count == 0) {
                std::cout << "최소 1장 촬영하세요." << std::endl;
                continue;
            }
            std::cout << count << "장으로 등록 완료." << std::endl;
            break;
        }
    }

    cv::destroyAllWindows();
    std::cout << name << " 등록 완료! (" << count << "장 저장)" << std::endl;
}

void trainModel() {
    std::cout << "\n모델 학습 시작..." << std::endl;

    std::string faces_dir  = BASE_PATH + "/faces";
    std::string model_path = BASE_PATH + "/face_model.yml";
    std::string name_path  = BASE_PATH + "/name_map.txt";

    if (!fs::exists(faces_dir) || fs::is_empty(faces_dir)) {
        std::cerr << "faces/ 폴더가 없거나 비어있습니다!" << std::endl;
        return;
    }

    auto recognizer = cv::face::LBPHFaceRecognizer::create();
    std::vector<cv::Mat> images;
    std::vector<int> labels;
    std::map<int, std::string> id_to_name;

    int id = 0;
    for (auto& person_dir : fs::directory_iterator(faces_dir)) {
        if (!person_dir.is_directory()) continue;

        std::string person_name = person_dir.path().filename().string();
        id_to_name[id] = person_name;
        int img_count = 0;

        for (auto& img_file : fs::directory_iterator(person_dir.path())) {
            if (img_file.path().extension() != ".jpg") continue;

            cv::Mat img = cv::imread(
                img_file.path().string(), cv::IMREAD_GRAYSCALE);
            if (img.empty()) continue;

            cv::resize(img, img, cv::Size(100, 100));
            images.push_back(img);
            labels.push_back(id);
            img_count++;
        }

        std::cout << " - " << person_name
                  << ": " << img_count << "장 로드" << std::endl;
        id++;
    }

    if (images.empty()) {
        std::cerr << "학습할 이미지가 없습니다!" << std::endl;
        return;
    }

    recognizer->train(images, labels);
    recognizer->save(model_path);

    std::ofstream name_file(name_path);
    for (auto& pair : id_to_name) {
        name_file << pair.first << " " << pair.second << "\n";
    }

    std::cout << "\n학습 완료!" << std::endl;
    std::cout << "총 " << id << "명, " << images.size() << "장" << std::endl;
    std::cout << "저장 위치: " << model_path << std::endl;
    std::cout << "저장 위치: " << name_path << std::endl;
}

void listRegistered() {
    std::string faces_dir = BASE_PATH + "/faces";
    std::cout << "\n=== 등록된 사람 목록 ===" << std::endl;

    if (!fs::exists(faces_dir)) {
        std::cout << "등록된 사람 없음" << std::endl;
        return;
    }

    int total = 0;
    for (auto& person_dir : fs::directory_iterator(faces_dir)) {
        if (!person_dir.is_directory()) continue;

        std::string name = person_dir.path().filename().string();
        int count = 0;
        for (auto& f : fs::directory_iterator(person_dir.path())) {
            if (f.path().extension() == ".jpg") count++;
        }
        std::cout << " - " << name << ": " << count << "장" << std::endl;
        total++;
    }

    if (total == 0) std::cout << "등록된 사람 없음" << std::endl;
    else std::cout << "총 " << total << "명 등록됨" << std::endl;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    camera_node = std::make_shared<CameraNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(camera_node);

    std::thread ros_thread([&]() {
        executor.spin();
    });

    if (!initDetector()) {
        executor.cancel();
        if (ros_thread.joinable()) ros_thread.join();
        camera_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    while (rclcpp::ok()) {
        std::cout << "\n=============================" << std::endl;
        std::cout << " 1. 얼굴 등록" << std::endl;
        std::cout << " 2. 모델 학습" << std::endl;
        std::cout << " 3. 등록 목록 확인" << std::endl;
        std::cout << " 0. 종료" << std::endl;
        std::cout << "=============================" << std::endl;
        std::cout << "선택: ";

        int choice;
        std::cin >> choice;

        switch (choice) {
            case 1: registerFace();   break;
            case 2: trainModel();     break;
            case 3: listRegistered(); break;
            case 0:
                std::cout << "종료합니다." << std::endl;
                executor.cancel();
                if (ros_thread.joinable()) ros_thread.join();
                camera_node.reset();
                rclcpp::shutdown();
                return 0;
            default:
                std::cout << "잘못된 입력입니다." << std::endl;
        }
    }

    executor.cancel();
    if (ros_thread.joinable()) ros_thread.join();
    camera_node.reset();
    rclcpp::shutdown();
    return 0;
}