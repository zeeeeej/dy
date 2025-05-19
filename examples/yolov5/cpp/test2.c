#include <stdio.h>
#include "../hd_uart_parser.h"


static void callback(const unsigned char *action_id, size_t action_id_size) {
    // 收到action_id
    printf("---->收到回调！！！\n");
    //关联图片和action_id
}

int main(int argc, char *argv[]) {
    hd_uart_init(callback);
    return 0;
}