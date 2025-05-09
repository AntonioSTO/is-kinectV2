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

// Estrutura para armazenar um conjunto de imagens
struct FrameTriple {
    cv::Mat rgb;
    cv::Mat ir;
    cv::Mat depth;
    int index;
};

// Fila e variáveis de sincronização
std::queue<FrameTriple> frame_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::atomic<bool> done(false);

// Função para salvar frames
void save_frame(const FrameTriple& frame, const std::string& base_path) {
    std::string rgb_filename = base_path + "/RGB_imgs/rgb_frame_" + std::to_string(frame.index) + ".jpg";
    std::string ir_filename = base_path + "/IR_imgs/ir_frame_" + std::to_string(frame.index) + ".jpg";
    std::string depth_filename = base_path + "/Depth_Maps/depth_frame_" + std::to_string(frame.index) + ".jpg";

    cv::imwrite(rgb_filename, frame.rgb);
    cv::imwrite(ir_filename, frame.ir);
    cv::imwrite(depth_filename, frame.depth);
}

// Função de thread para consumir frames da fila e salvar
void save_frames_thread(const std::string& base_path) {
    while (!done || !frame_queue.empty()) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [] { return !frame_queue.empty() || done; });

        if (!frame_queue.empty()) {
            FrameTriple frame = frame_queue.front();
            frame_queue.pop();
            lock.unlock();  // Libera mutex enquanto salva imagem

            save_frame(frame, base_path);
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

    libfreenect2::SyncMultiFrameListener listener(
        libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    // Caminhos
    std::string base_path = "imagens_kinect/p" + std::to_string(person_id) + "g" + std::to_string(gesture_id);
    fs::create_directories(base_path + "/RGB_imgs");
    fs::create_directories(base_path + "/IR_imgs");
    fs::create_directories(base_path + "/Depth_Maps");

    // Threads de salvamento
    const int num_threads = 8;
    std::vector<std::thread> saver_threads;
    for (int i = 0; i < num_threads; ++i) {
        saver_threads.push_back(std::thread(save_frames_thread, base_path));
    }

    bool capturing = false;
    int frame_index = 0;
    std::cout << "Pressione 's' para capturar, 'q' para sair." << std::endl;

    while (true) {
        libfreenect2::FrameMap frames;
        listener.waitForNewFrame(frames);

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        // Converte RGB
        cv::Mat rgbMat(rgb->height, rgb->width, CV_8UC4, rgb->data);

        // Converte IR
        cv::Mat irMat(ir->height, ir->width, CV_32FC1, ir->data);
        cv::Mat irVis;
        irMat.convertTo(irVis, CV_8UC1, 255.0 / 65535.0);

        // Converte Depth
        cv::Mat depthMat(depth->height, depth->width, CV_32FC1, depth->data);
        cv::Mat depthVis;
        cv::normalize(depthMat, depthVis, 0, 255, cv::NORM_MINMAX);
        depthVis.convertTo(depthVis, CV_8UC1);

        // Mostra imagens
        cv::imshow("RGB", rgbMat);
        cv::imshow("IR", irVis);
        cv::imshow("Depth", depthVis);

        char key = cv::waitKey(1);
        if (key == 's') capturing = true;
        if (key == 'q') break;

        if (capturing) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            frame_queue.push({
                rgbMat.clone(),
                irVis.clone(),
                depthVis.clone(),
                frame_index++
            });
            queue_cv.notify_one();
            capturing = false;
        }

        listener.release(frames);
    }

    dev->stop();
    dev->close();

    done = true;
    queue_cv.notify_all();
    for (auto& t : saver_threads) t.join();

    std::cout << "Captura e salvamento concluídos com sucesso!" << std::endl;
    return 0;
}
