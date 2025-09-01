#!/bin/bash

# hd_sync - 同步SRC工程目录到DST工程目录的特定文件（非递归）

# 显示使用说明
usage() {
    echo "使用方法: $0 SRC DST"
    echo "  SRC - 源工程目录"
    echo "  DST - 目标工程目录"
    exit 1
}

# 检查参数数量
if [ $# -ne 2 ]; then
    echo "错误: 需要两个参数"
    usage
fi

SRC="$1"
DST="$2"

# 检查源目录是否存在
if [ ! -d "$SRC" ]; then
    echo "错误: 源目录 '$SRC' 不存在"
    exit 1
fi

# 检查目标目录是否存在，如果不存在则创建
if [ ! -d "$DST" ]; then
    echo "注意: 目标目录 '$DST' 不存在，正在创建..."
    mkdir -p "$DST"
fi

# 函数：复制指定目录下的文件（不递归）
copy_files() {
    local src_dir="$1"
    local dst_dir="$2"
    local pattern="$3"
    
    # 创建目标目录（如果不存在）
    mkdir -p "$dst_dir"
    
    # 检查源目录是否存在
    if [ ! -d "$src_dir" ]; then
        echo "警告: 源目录 '$src_dir' 不存在"
        return 1
    fi
    
    # 切换到源目录
    pushd "$src_dir" > /dev/null
    
    # 查找匹配的文件
    local files=($(ls $pattern 2>/dev/null || true))
    
    if [ ${#files[@]} -eq 0 ]; then
        echo "警告: 在 $src_dir 中没有找到匹配 $pattern 的文件"
        popd > /dev/null
        return 1
    fi
    
    for file in "${files[@]}"; do
        if [ -f "$file" ]; then
            echo "复制: $src_dir/$file 到 $dst_dir"
            cp "$file" "$dst_dir"
        fi
    done
    
    popd > /dev/null
    return 0
}

echo ">>>>>>>>>>>>>>>>>>>开始复制"

# 复制SRC/include/hd_uart/* 至 DST/include/hd_uart
echo "复制文件SRC/include/hd_uart/* 至 DST/hd_uart/include"
copy_files "$SRC/include/hd_uart" "$DST/hd_uart/include" "*"

# 复制SRC/include/shell/* 至 DST/include/hd_uart
echo "复制文件SRC/include/shell/* 至 DST/hd_uart/include"
copy_files "$SRC/include/shell" "$DST/hd_uart/include" "*"

# 复制SRC/src/*.c 至 DST/src/hd_uart
echo "复制文件SRC/src/*.c 至 DST/hd_uart/src"
copy_files "$SRC/src" "$DST/hd_uart/src" "*.c"

# 复制SRC/src/shell/*.c 至 DST/src
echo "复制文件SRC/src/shell/*.c 至 DST/hd_uart/src"
copy_files "$SRC/src/shell" "$DST/hd_uart/src" "*.c"

echo "<<<<<<<<<<<<<<<<<<<<复制完毕"