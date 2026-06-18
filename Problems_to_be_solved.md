# 待解决问题事项

## 1 编译 EtherCat 的 cmake 版本不对
编译 EtherCat , cmake 版本 要求 3.28 , 目标机只有 3.16 需要修改文件 才可以编译, 后续考虑升级 目标机的cmake 版本,需要机的修改 CMakeLists.txt .
~~~bash
core/src/drivers/plugins/native/ethercat/libs/soem/CMakeLists.txt
cmake_minimum_required(VERSION 3.28)  # 根据需要修改指定版本
```