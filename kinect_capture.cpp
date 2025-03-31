#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
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
   
    std::cout << "Pressione 's' para iniciar a captura contínua ou 'q' para sair." << std::endl;
   
    while (true) {
        listener.waitForNewFrame(frames);
       
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        // Converter para OpenCV Mat
        cv::Mat rgbMat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        cv::Mat depthMat(depth->height, depth->width, CV_32FC1, depth->data);

        // Normalizar profundidade para visualização
        cv::Mat depthVis;
        depthMat.convertTo(depthVis, CV_8UC1, 255.0 / 4500);

        // Exibir as imagens
        cv::imshow("RGB", rgbMat);
        cv::imshow("Depth", depthVis);

        // Captura contínua após pressionar 's'
        char key = cv::waitKey(1);
        if (key == 's') {
            capturing = true;
            std::cout << "Captura iniciada! Pressione 'q' para parar." << std::endl;
        }
        if (capturing) {
            std::string rgb_filename = "rgb_frame_" + std::to_string(frame_count) + ".png";
            std::string depth_filename = "depth_frame_" + std::to_string(frame_count) + ".png";
           
            cv::imwrite(rgb_filename, rgbMat);
            cv::imwrite(depth_filename, depthVis);
           
            std::cout << "Salvou: " << rgb_filename << " e " << depth_filename << std::endl;
            frame_count++;
        }
       
        if (key == 'q') break;

        listener.release(frames);
    }

    dev->stop();
    dev->close();

    return 0;
}