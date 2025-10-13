# Gtest and Gmock Learn
## Environment setup
1. install gtest
   
   ```c
      pacman -S mingw-w64-ucrt-x86_64-gtest
   ```

2. install gcovr    
   ```python
      pip install gcovr
   ```

3. CMake Library output directory Set  

   1. CMAKE_ARCHIVE_OUTPUT_DIRECTORY：默认存放静态库的文件夹位置
   2. CMAKE_LIBRARY_OUTPUT_DIRECTORY：默认存放动态库的文件夹位置
   3. LIBRARY_OUTPUT_PATH：默认存放库文件的位置，如果产生的是静态库并且没有指定 
   4. CMAKE_ARCHIVE_OUTPUT_DIRECTORY 则存放在该目录下，动态库也类似
   5. CMAKE_RUNTIME_OUTPUT_DIRECTORY：存放可执行软件的目录

```c
#ifdefined DLL_EXPORTS
   #ifdefined INSIDE_DLL
      #define SIMPLE_CLASS_EXPORT__declspec(dllexport)
   #else
     #define SIMPLE_CLASS_EXPORT__declspec(dllimport)
   #endif
   #else
      #define SIMPLE_CLASS_EXPORT
#endif 
```  
```cmake
   ctest -V
   ctest -T coverage
```
## how to use gtest
1. git clone this repository
2. cd to the repository directory
3. with vscode cmake configure and build
4. under build folder
   ```
   cd build
   # build cmake projects
   cmake ..             # Generate native build scripts for GoogleTest.
   # cmake clean with ninja
   ninja clean
   # cmake build with ninja
   ninja all
   #cTest with ninja
   ninja test
   ctest -V
   #cTest coverage with ninja
   ninja coverage
   ctest -T coverage
   #coverage report with ninja
   ninja coverage_report
   ```
5. vscode Testing
in vscode, you can use the `Testing` extension to run the tests.
after run the tests, you can use the `Testing` extension to view the test results.
shortcut key: 
ctrl+;+ctrl+o