#include <iostream>
#include <csignal>
#include <execinfo.h>
#include <cstdlib>
#include <dlfcn.h>
#include <rclcpp/rclcpp.hpp>
#include "udp_communication.hpp"

void *g_handle = nullptr;
void (*g_stop_server)() = nullptr;

// 종료 시 실행될 함수
void cleanup() {
    if (g_stop_server) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping UDP server...");
        g_stop_server();
    }
    if (g_handle) {
        dlclose(g_handle);
        g_handle = nullptr;
    }
    rclcpp::shutdown();
}

void signal_handler(int signal) {
    void *array[50];
    size_t size = backtrace(array, 50);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Error: Signal received : %d", signal);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Stack trace (with source line info):");

    char exe_path[256];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len != -1) {
        exe_path[len] = '\0';
        for (size_t i = 0; i < size; i++) {
            char command[512];
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"addr2line -e %s -f -p %p", exe_path, array[i]);
            int result = system(command);
            if (result != 0) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Command failed with result: %d", result);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Error: %s", strerror(errno));  // 실패 원인 출력
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Command executed successfully.");
            }
        }
    }

    cleanup();
    exit(1);
    return;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    g_handle = dlopen("/home/airbot/airbot_ws/install/udp_interface/include/udp_interface/libNetwork.so", RTLD_LAZY);
    if (!g_handle) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Cannot open library: %s", dlerror());
        return -1;
    }

    auto start_server = (void(*)())dlsym(g_handle, "start_server");
    g_stop_server = (void(*)())dlsym(g_handle, "stop_server");
    auto APIGetVersion = (void(*)(std::string&))dlsym(g_handle, "APIGetVersion");
    auto APISetEnc = (void(*)(bool))dlsym(g_handle, "APISetEnc");

    if (!start_server || !g_stop_server || !APIGetVersion || !APISetEnc) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Cannot open library: %s", dlerror());
        cleanup();
        return -1;
    }

    start_server();
    std::string version;
    APIGetVersion(version);
    APISetEnc(false);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"API Version: %s", version.c_str());

    std::atexit(cleanup);

    auto udp_node = std::make_shared<UdpCommunication>();
    rclcpp::spin(udp_node);

    cleanup();
    return 0;
}