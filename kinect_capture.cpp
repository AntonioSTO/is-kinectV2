#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    libfreenect2::Freenect2 freenect2;
    if (freenect2.enumerateDevices() == 0) {
        std::cerr << "Nenhum Kinect v2 encontrado!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    libfreenect2::Freenect2Device *dev = freenect2.openDevice(serial);

    if (!dev) {
        std::cerr << "Falha ao abrir dispositivo." << std::endl;
        return -1;
    }

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->start()) {
        std::cerr << "Falha ao iniciar o dispositivo." << std::endl;
        return -1;
    }

    libfreenect2::Freenect2Device::IrCameraParams irParams = dev->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev->getColorCameraParams();

    libfreenect2::Registration registration(irParams, colorParams);
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    std::cout << "Pressione Ctrl+C para sair." << std::endl;

    pcl::visualization::CloudViewer viewer("Nuvem de Pontos RGB-D");

    while (!viewer.wasStopped()) {
        libfreenect2::FrameMap frames;
        if (!listener.waitForNewFrame(frames, 10 * 1000)) {
            std::cerr << "Timeout esperando novo frame!" << std::endl;
            break;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        if (!rgb || !depth) {
            std::cerr << "Frames nulos recebidos!" << std::endl;
            listener.release(frames);
            continue;
        }

        registration.apply(rgb, depth, &undistorted, &registered);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->width = 512;
        cloud->height = 424;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (int y = 0; y < 424; ++y) {
            for (int x = 0; x < 512; ++x) {
                float rx, ry, rz;
                registration.getPointXYZRGB(&undistorted, &registered, x, y, rx, ry, rz);

                pcl::PointXYZRGB &pt = cloud->at(x, y);
                pt.x = rx;
                pt.y = ry;
                pt.z = rz;

                uint32_t rgb_val = *reinterpret_cast<uint32_t*>(&registered.data[y * 512 * 4 + x * 4]);
                pt.r = (rgb_val >> 16) & 0xFF;
                pt.g = (rgb_val >> 8) & 0xFF;
                pt.b = rgb_val & 0xFF;
            }
        }

        viewer.showCloud(cloud);
        listener.release(frames);
    }

    dev->stop();
    dev->close();

    return 0;
}
