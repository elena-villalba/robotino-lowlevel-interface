#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include "rec/robotino/api2/all.h"
#include "rec/robotino/api2/OmniDriveModel.h"
#include "rec/robotino/api2/OmniDrive.h"
#include "rec/robotino/api2/Odometry.h"
#include "rec/robotino/api2/MotorArray.h"

rec::robotino::api2::Com com;
rec::robotino::api2::OmniDriveModel omniDriveModel;
rec::robotino::api2::OmniDrive omniDrive;
rec::robotino::api2::Odometry odometry;
rec::robotino::api2::MotorArray motorArray;

struct VelCmd {
    float vx;
    float vy;
    float w;
};

struct OdomData {
    float x;
    float y;
    float theta;
};

std::mutex data_mutex;
VelCmd latest_cmd = {0.0f, 0.0f, 0.0f};
auto last_cmd_time = std::chrono::steady_clock::now();
std::atomic<bool> running(true);

void receive_loop(int connfd) {
    while (running) {
        VelCmd cmd;
        ssize_t bytes = recv(connfd, &cmd, sizeof(cmd), MSG_WAITALL);
        if (bytes != sizeof(cmd)) {
            std::cerr << "[RX] Error or disconnect while receiving VelCmd (received: " << bytes << " bytes)." << std::endl;
            running = false;
            break;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            latest_cmd = cmd;
            last_cmd_time = std::chrono::steady_clock::now();
        }
        std::cout << "[RX] Cmd: vx=" << cmd.vx << ", vy=" << cmd.vy << ", w=" << cmd.w << std::endl;
    }
}

void monitor_loop() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(data_mutex);
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time);
        if (duration.count() > 100) {
            std::cout << "[ALERT] No command received for " << duration.count()
                      << " ms. Last: vx=" << latest_cmd.vx
                      << ", vy=" << latest_cmd.vy << ", w=" << latest_cmd.w << std::endl;
        }
    }
}

void send_loop(int connfd) {
    while (running) {
        try {
            double x, y, phi;
            odometry.readings(&x, &y, &phi);

            OdomData odom{static_cast<float>(x), static_cast<float>(y), static_cast<float>(phi)};

            ssize_t bytes = send(connfd, &odom, sizeof(odom), 0);
            if (bytes != sizeof(odom)) {
                std::cerr << "[TX] Error sending odometry (bytes sent: " << bytes << ")." << std::endl;
                running = false;
                break;
            }
        } catch (const std::exception& e) {
            std::cerr << "[TX] Exception: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void control_loop() {
    while (running) {
        VelCmd cmd;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            cmd = latest_cmd;
        }
        omniDrive.setVelocity(cmd.vx, cmd.vy, cmd.w);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void log_motor_velocities_loop() {
    std::ofstream file("velocidades_" + std::to_string(time(nullptr)) + ".csv", std::ios::app);
    if (!file.is_open()) {
        std::cerr << "[LOG] Could not open velocity log file." << std::endl;
        return;
    }

    auto start_time = std::chrono::steady_clock::now();
    std::vector<float> velocities(3);

    while (running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        try {
            motorArray.actualVelocities(velocities.data());
        } catch (const rec::robotino::api2::RobotinoException& e) {
            std::cerr << "[LOG] Error getting motor velocities: " << e.what() << std::endl;
            continue;
        }

        file << elapsed_ms;
        for (float v : velocities) {
            file << "," << v;
        }
        file << std::endl;
        file.flush();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    file.close();
}

int main(int argc, char **argv) {
    std::string hostname = "127.0.0.1";
    if (argc > 1) hostname = argv[1];

    com.setAddress(hostname.c_str());
    com.connectToServer(true);
    if (!com.isConnected()) {
        std::cerr << "[ERROR] Could not connect to Robotino." << std::endl;
        return -1;
    }

    omniDriveModel.setRb(0.18);
    omniDriveModel.setRw(0.05);
    omniDriveModel.setGear(32);

    omniDrive.setComId(com.id());
    odometry.setComId(com.id());

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(9000);

    if (bind(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("bind");
        return 1;
    }

    listen(sockfd, 1);
    std::cout << "Waiting for TCP connection on port 9000..." << std::endl;

    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int connfd = accept(sockfd, (sockaddr*)&client_addr, &client_len);
    if (connfd < 0) {
        perror("accept");
        return 1;
    }
    std::cout << "Client connected." << std::endl;

    std::thread rx_thread(receive_loop, connfd);
    std::thread tx_thread(send_loop, connfd);
    std::thread monitor_thread(monitor_loop);
    std::thread control_thread(control_loop);
    std::thread log_thread(log_motor_velocities_loop);

    rx_thread.join();
    tx_thread.join();
    monitor_thread.join();
    control_thread.join();
    log_thread.join();

    close(connfd);
    close(sockfd);
    com.disconnectFromServer();
    rec::robotino::api2::shutdown();

    return 0;
}
