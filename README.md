# manipulator_line_reconstruction

本程序主要有三个线程
- main()：运行主程序，处理接收到的信息，进行运动控制等
- thread1: 主要与机械手从手进行串口通讯
- thread2: 主要与其他部分进行通讯，包括脑电控制、视觉等


