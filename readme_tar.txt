tar cvf ../libs.tar -C ./ $(find . -type f -name "*.so*" -o -type l)
