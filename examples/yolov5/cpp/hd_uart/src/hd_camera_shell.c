
#include "hd_camera_shell.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>

#define  TIMEOUT        3

int hd_camera_shell_init(uint8_t addr) {
    printf("hd_camera_shell_init <1.0> %d\n", addr);
    return 0;
}

int hd_camera_shell_exec(const char *shell, char *shell_result) {
    if (!shell || !shell_result) return 1;

    int pipefd[2];
    if (pipe(pipefd)) {
        perror("pipe");
        return 2;
    }

    pid_t pid = fork();
    if (pid == -1) {
        perror("fork");
        close(pipefd[0]);
        close(pipefd[1]);
        return 3;
    } else if (pid == 0) { // 子进程
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);

        execl("/bin/sh", "sh", "-c", shell, (char *) NULL);
        perror("exec failed");
        _exit(127); // 127 是 "command not found" 的标准退出码
    } else {

        // 父进程
        close(pipefd[1]);

        // 非阻塞读取管道
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(pipefd[0], &fds);

        struct timeval tv = {.tv_sec = TIMEOUT, .tv_usec = 0};
        ssize_t bytes_read;
        size_t total_bytes = 0;
        char buffer[256];
        printf("select start!\n");
        while (1) {
            int ready = select(pipefd[0] + 1, &fds, NULL, NULL, &tv);
            if (ready == -1) {
                perror("select");
                break;
            } else if (ready == 0) {
                // 超时发生
                kill(pid, SIGKILL);
                waitpid(pid, NULL, 0);
                strcpy(shell_result, "Command timed out");
                close(pipefd[0]);
                return -2;
            }

            bytes_read = read(pipefd[0], buffer, sizeof(buffer));
            if (bytes_read <= 0) break;

            if (total_bytes + bytes_read < MAX_RESULT_LENGTH - 1) {
                memcpy(shell_result + total_bytes, buffer, bytes_read);
                total_bytes += bytes_read;
            } else {
                break; // 缓冲区满
            }
        }
        printf("select end!\n");
        shell_result[total_bytes] = '\0';
        close(pipefd[0]);

        // 取消超时
        // alarm(0);

        // 等待子进程结束
        int status;
        printf("waitpid start ...\n");
        waitpid(pid, &status, 0);
        printf("waitpid end!\n");
        printf("result=%s", shell_result);
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }
}

void hd_camera_shell_deinit() {
    printf("hd_camera_shell_deinit \n");
}