#!/bin/bash

# 定义一个存放字符串的数组
example_array=(
    "examples\i2c_tools" 
    "examples\nvs_rw_blob"
)

# 使用循环遍历数组并输出每个字符串
for element in "${example_array[@]}"; do

    echo "Building project at [$element]"
    cd $element
    idf.py set-target esp32p4
    idf.py build
    cd -
done
