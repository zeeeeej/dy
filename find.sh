for lib in examples/yolov5/cpp/libs/*.so*; do
    if nm -D "$lib" 2>/dev/null | grep -q rkipc_version_dump; then
        echo "Found in $lib"
        nm -D "$lib" | grep rkipc_version_dump
    fi
done

