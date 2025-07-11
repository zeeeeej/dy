#ifndef H__HD_CAMERA_SHELL__H
#define H__HD_CAMERA_SHELL__H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_COMMAND_LENGTH 1024
#define MAX_RESULT_LENGTH 10240

// * 1.接受[485串口]发来的shell命令
// * 2.执行shell命令
// * 3.将命令返回[485串口]

int hd_camera_shell_init(uint8_t addr);

/**
 *
 * @param shell 执行的命令字符串
 * @param shell_result 执行命令的结果字符串
 * @return  -2: 命令执行超时 127: 命令未找到（子进程返回）0: 成功
 */
int hd_camera_shell_exec(
        const char *shell,
        char *shell_result
);

void hd_camera_shell_deinit();

#ifdef __cplusplus
}
#endif

#endif // H__HD_CAMERA_SHELL__H
