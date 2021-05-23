# osqp中问题笔记
- 在cmake语法中，link_libraries和target_link_libraries是很重要的两个链接库的方式，虽然写法上很相似，但是功能上有很大区别：
    + ```
    [100%] Linking CXX executable main
    CMakeFiles/main.dir/main.cc.o: In function `main':
    main.cc:(.text+0x226): undefined reference to `csc_matrix'
    main.cc:(.text+0x27f): undefined reference to `csc_matrix'
    main.cc:(.text+0x2c7): undefined reference to `osqp_set_default_settings'
    main.cc:(.text+0x2fb): undefined reference to `osqp_setup'
    main.cc:(.text+0x30a): undefined reference to `osqp_solve'
    collect2: error: ld returned 1 exit status
    CMakeFiles/main.dir/build.make:83: recipe for target 'main' failed
    make[2]: *** [main] Error 1
    CMakeFiles/Makefile2:72: recipe for target 'CMakeFiles/main.dir/all' failed
    make[1]: *** [CMakeFiles/main.dir/all] Error 2

    ```
    + 1 link_libraries用在add_executable之前，target_link_libraries用在add_executable之后
    + 2 link_libraries用来链接静态库，target_link_libraries用来链接导入库，即按照header file + .lib + .dll方式隐式调用动态库的.lib库