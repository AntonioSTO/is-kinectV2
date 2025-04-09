#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

namespace fs = std::filesystem;

// Estrutura para armazenar um par de imagens
struct FramePair {
    cv::Mat rgb;
    cv::Mat depth;
    int index;
};

// Fila e variáveis de sincronização
std::queue<FramePair> frame_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::atomic<bool> done(false);

// Função para salvar os frames em background
void save_frames_thread(const std::string& base_path) {
    int counter = 0;
    while (!done || !frame_queue.empty()) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [] { return !frame_queue.empty() || done; });

        while (!frame_queue.empty()) {
            FramePair frame = frame_queue.front();
            frame_queue.pop();
            lock.unlock();  // Libera mutex enquanto salva imagem (operação pesada)

            std::string rgb_filename = base_path + "/RGB_imgs/rgb_frame_" + std::to_string(frame.index) + ".png";
            std::string depth_filename = base_path + "/Depth_Maps/depth_frame_" + std::to_string(frame.index) + ".png";

            cv::imwrite(rgb_filename, frame.rgb);
            cv::imwrite(depth_filename, frame.depth);

            lock.lock(); // Rebloqueia para continuar o loop
        }
    }
}

int main(int argc, char** argv) {
    int person_id = 1, gesture_id = 1;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-p" && i + 1 < argc) {
            person_id = std::stoi(argv[i + 1]);
        } else if (std::string(argv[i]) == "-g" && i + 1 < argc) {
            gesture_id = std::stoi(argv[i + 1]);
        }
    }

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();
    if (!dev) {
        std::cerr << "Erro ao abrir dispositivo Kinect!" << std::endl;
        return -1;
    }

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    // Caminhos
    std::string base_path = "imagens_kinect/p" + std::to_string(person_id) + "g" + std::to_string(gesture_id);
    fs::create_directories(base_path + "/RGB_imgs");
    fs::create_directories(base_path + "/Depth_Maps");

    // Inicia thread de salvamento
    std::thread saver_thread(save_frames_thread, base_path);

    bool capturing = false;
    int frame_index = 0;
    std::cout << "Pressione 's' para iniciar/parar a captura ou 'q' para sair." << std::endl;

    while (true) {
        libfreenect2::FrameMap frames;
        listener.waitForNewFrame(frames);

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat rgbMat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        cv::Mat depthMat(depth->height, depth->width, CV_32FC1, depth->data);

        cv::Mat rgbResized;
        cv::resize(rgbMat, rgbResized, cv::Size(1280, 720));

        cv::Mat depthVis;
        depthMat.convertTo(depthVis, CV_8UC1, 255.0 / 4500.0);

        cv::imshow("Depth", depthVis);
        cv::imshow("RGB", rgbResized);

        char key = cv::waitKey(1);
        if (key == 's') capturing = !capturing;
        if (key == 'q') break;

        if (capturing) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            frame_queue.push({ rgbResized.clone(), depthVis.clone(), frame_index++ });
            queue_cv.notify_one();
        }

        listener.release(frames);
    }

    // Finalização
    dev->stop();
    dev->close();

    // Sinaliza para encerrar a thread de salvamento
    done = true;
    queue_cv.notify_all();
    saver_thread.join();

    std::cout << "Captura e salvamento concluídos com sucesso!" << std::endl;
    return 0;
}
