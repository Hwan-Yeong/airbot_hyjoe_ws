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
        std::cout << "Stopping UDP server..." << std::endl;
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

    std::cerr << "\nError: Signal " << signal << " received.\n";
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    std::cerr << "Stack trace (with source line info):\n";

    char exe_path[256];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len != -1) {
        exe_path[len] = '\0'; // Ensure null termination
        for (size_t i = 0; i < size; i++) {
            char command[512];
            snprintf(command, sizeof(command), "addr2line -e %s -f -p %p", exe_path, array[i]);
            system(command);
        }
    }

    cleanup();
    exit(1);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    g_handle = dlopen("/home/airbot/airbot_ws/install/udp_interface/include/udp_interface/libNetwork.so", RTLD_LAZY);
    if (!g_handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return -1;
    }

    auto start_server = (void(*)())dlsym(g_handle, "start_server");
    g_stop_server = (void(*)())dlsym(g_handle, "stop_server");
    auto APIGetVersion = (void(*)(std::string&))dlsym(g_handle, "APIGetVersion");
    auto APISetEnc = (void(*)(bool))dlsym(g_handle, "APISetEnc");

    if (!start_server || !g_stop_server || !APIGetVersion || !APISetEnc) {
        std::cerr << "Cannot load symbols: " << dlerror() << '\n';
        cleanup();
        return -1;
    }

    start_server();  // UDP 서버 시작
    std::string version;
    APIGetVersion(version);
    APISetEnc(false);
    std::cout << "API Version: " << version << std::endl;

    std::atexit(cleanup); // 정상 종료 시 cleanup 실행

    //std::signal(SIGINT, signal_handler);
    //std::signal(SIGSEGV, signal_handler);
    //std::signal(SIGABRT, signal_handler);
    //std::signal(SIGFPE, signal_handler);

    auto udp_node = std::make_shared<UdpCommunication>();
    rclcpp::spin(udp_node);
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(udp_node);
    // executor.add_node(state_node);
    // executor.spin();

    cleanup();
    return 0;
}