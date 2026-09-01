/**
 * @file main_debug.c
 * @brief OpenPLC CANopen 插件独立调试主程序
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>

#include "../canopen_plugin.h"

// 运行控制标志位
static volatile bool g_keep_running = true;

// 信号处理函数，用于捕获 Ctrl+C (SIGINT) 和 SIGTERM，优雅退出主循环
static void handle_signal(int sig)
{
    (void)sig;
    printf("\n[DEBUG MAIN] 接收到退出信号，即将中断主循环...\n");
    g_keep_running = false;
}

int main(int argc, char *argv[])
{
    // 1. 注册信号句柄
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("===================================================\n");
    printf("   OpenPLC CANopen 插件 Lifecycle 调试测试程序    \n");
    printf("===================================================\n\n");

    // 2. 构造插件运行参数
    plugin_runtime_args_t runtime_args;
    memset(&runtime_args, 0, sizeof(runtime_args));

    // 设置配置文件路径（可通过命令行参数 1 指定，未指定则使用默认路径）
    if (argc > 1 && argv[1] != NULL && argv[1][0] != '\0')
    {
        snprintf(runtime_args.plugin_specific_config_file_path,
                 sizeof(runtime_args.plugin_specific_config_file_path),
                 "%s", argv[1]);
        printf("[DEBUG MAIN] 配置文件路径: %s (来自命令行参数)\n",
               runtime_args.plugin_specific_config_file_path);
    }
    else
    {
        snprintf(runtime_args.plugin_specific_config_file_path,
                 sizeof(runtime_args.plugin_specific_config_file_path),
                 "./canopen_config.json");
        printf("[DEBUG MAIN] 配置文件路径: %s (默认路径)\n",
               runtime_args.plugin_specific_config_file_path);
    }

    // 3. 执行初始配置 - int init(void *args) [初始执行一次]
    printf("\n[DEBUG MAIN] ---> [1/5] 执行 init(void *args)\n");
    int init_status = init(&runtime_args);
    if (init_status != 0)
    {
        fprintf(stderr, "[DEBUG MAIN] 错误: init() 执行失败，返回值: %d\n", init_status);
        return EXIT_FAILURE;
    }
    printf("[DEBUG MAIN] init() 执行成功。\n");

    // 4. 执行循环启动 - int start_loop(void) [循环开始执行一次]
    printf("\n[DEBUG MAIN] ---> [2/5] 执行 start_loop(void)\n");
    int start_status = start_loop();
    if (start_status != 0)
    {
        fprintf(stderr, "[DEBUG MAIN] 错误: start_loop() 执行失败，返回值: %d\n", start_status);
        printf("[DEBUG MAIN] 执行 cleanup() 并退出...\n");
        cleanup();
        return EXIT_FAILURE;
    }
    printf("[DEBUG MAIN] start_loop() 执行成功，后台接收/工作线程已启动。\n");

    // 5. 循环部分 - 模拟主线程/PLC Cycle 周期调度
    printf("\n[DEBUG MAIN] ---> [3/5] 进入周期运行循环 (按 Ctrl+C 触发 stop_loop & cleanup)\n");
    unsigned long cycle_counter = 0;
    const useconds_t cycle_period_us = 10000; // 10ms (100Hz) 周期

    while (g_keep_running)
    {
        cycle_counter++;

        // 5.1 开始循环周期: void cycle_start(void)
        cycle_start();

        // 5.2 循环结束逻辑: void cycle_end(void) (处理 lifecycle tick, 节点状态同步, PDO/SDO等)
        cycle_end();

        // 定期打印运行状态 (约每 1 秒/100 次周期打印一次)
        if (cycle_counter % 100 == 0)
        {
            printf("[DEBUG MAIN] [周期 #%lu] cycle_start/cycle_end 持续调度中...\n", cycle_counter);
        }

        // 间隔等待，模拟控制周期
        usleep(cycle_period_us);
    }

    printf("\n[DEBUG MAIN] 退出周期循环，共计完成 %lu 次周期。\n", cycle_counter);

    // 6. 停止循环 - void stop_loop(void)
    printf("\n[DEBUG MAIN] ---> [4/5] 执行 stop_loop(void)\n");
    stop_loop();
    printf("[DEBUG MAIN] stop_loop() 执行完成，后台线程及 Socket 已停止/关闭。\n");

    // 7. 清理资源 - void cleanup(void)
    printf("\n[DEBUG MAIN] ---> [5/5] 执行 cleanup(void)\n");
    cleanup();
    printf("[DEBUG MAIN] cleanup() 执行完成，CANopenNode 结构已销毁。\n");

    printf("\n===================================================\n");
    printf("   CANopen 插件调试运行结束，程序安全退出          \n");
    printf("===================================================\n");

    return EXIT_SUCCESS;
}
