#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <string>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    int person_id = 1; // Valor padrão
    int gesture_id = 1; // Valor padrão

    // Processar argumentos do terminal
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-p" && i + 1 < argc) {
            person_id = std::stoi(argv[i + 1]);
            i++; // Pular o próximo argumento, pois já foi processado
        } else if (arg == "-g" && i + 1 < argc) {
            gesture_id = std::stoi(argv[i + 1]);
            i++;
        }
    }

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;

    if (freenect2.enumerateDevices() == 0) {
        std::cerr << "Nenhum dispositivo encontrado!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDeviceSerialNumber(0);
    dev = freenect2.openDevice(serial);

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    libfreenect2::FrameMap frames;
    bool capturing = false;
    int frame_count = 0;

    std::string base_path = "imagens_kinect/";
    if (!std::filesystem::exists(base_path)) {
        std::filesystem::create_directory(base_path);
    }

    std::string person_gesture_folder = base_path + "p" + (person_id < 10 ? "00" : (person_id < 100 ? "0" : "")) + std::to_string(person_id) + "g" + (gesture_id < 10 ? "0" : "") + std::to_string(gesture_id);
    std::string rgb_path = person_gesture_folder + "/RGB_imgs/";
    std::string depth_path = person_gesture_folder + "/Depth_Maps/";

    std::filesystem::create_directories(rgb_path);
    std::filesystem::create_directories(depth_path);

    std::cout << "Capturando para p" << person_id << "g" << gesture_id << std::endl;
    std::cout << "Pressione 's' para iniciar a captura contínua ou 'q' para sair." << std::endl;

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();

        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat rgbMat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        cv::Mat depthMat(depth->height, depth->width, CV_32FC1, depth->data);
        cv::Mat depthVis;
        depthMat.convertTo(depthVis, CV_8UC1, 255.0 / 4500);

        cv::imshow("RGB", rgbMat);
        cv::imshow("Depth", depthVis);

        char key = cv::waitKey(1);
        if (key == 's') {
            capturing = true;
            std::cout << "Captura iniciada! Pressione 'q' para parar." << std::endl;
        }
        if (capturing) {
            std::string rgb_filename = rgb_path + "rgb_frame_" + std::to_string(frame_count) + ".png";
            std::string depth_filename = depth_path + "depth_frame_" + std::to_string(frame_count) + ".png";
            cv::imwrite(rgb_filename, rgbMat);
            cv::imwrite(depth_filename, depthVis);
            std::cout << "Salvou: " << rgb_filename << " e " << depth_filename << std::endl;
            frame_count++;
        }
        if (key == 'q') break;

        listener.release(frames);

        // Aguarda para manter a taxa de captura em 30 FPS
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds frame_duration(33);
        std::this_thread::sleep_for(frame_duration - std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time));
    }

    dev->stop();
    dev->close();

    return 0;
}
