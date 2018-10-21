#四旋翼飞行器控制板
#####湖北工业大学机械工程学院 周恒
#####GitHub：IZH20
* MCU stm32f427IIHX   168MHZ
* 接口映射表 
>1. IMU  (mpu6500+ist8310)
    PF7 --> SPI5_SCK
    PF8 --> SPI5_MISO
    PF9 --> SPI5_MOSI
>2. OLED 
    PA7 --> SPI1_MOSI
    PB3 --> SPI1_SCK
    PB4 --> SPI1_MISO
>2. 电调输出
    MOTOR1  --> PH10 -->TIM5_CH1
    MOTOR2  --> PH11 -->TIM5_CH2
    MOTOR3  --> PH12 -->TIM5_CH3
    MOTOR4  --> PI0  -->TIM5_CH4
>3. RC输入
    DBUS    --> PB7  -->USART1_RX
>4. 超声波
    TRIG    --> PH10
    ECHO    --> pI9  外部中断
>5. 按键
    s1      --> PD10
>6. LED指示灯
    LEDG(绿色) --> PF14
    LEDR(红色) --> PE11
    LED1(绿色) --> PG1
    LED2(绿色) --> PG2
    LED3(绿色) --> PG3
    LED4(绿色) --> PG4
    LED5(绿色) --> PG5
    LED6(绿色) --> PG6
    LED7(绿色) --> PG7
    LED8(绿色) --> PG8
>7. USART2(Extended,5v)  外接GPS
    USART2_TX --> PD5
    USART2_RX --> PD6
>8. USART3(Extended,3.3v) 备用串口
    USART3_TX --> PC10
    USART3_RX --> PC11
>9. USART6 用于山外上位机调试
    USART6_TX --> PG14
    USART6_RX --> PG9
###姿态解算
 >  要想写飞控，我们的第一步就是要获取imu传感器中的陀螺仪数据，加速度数据和磁力计数
    据一共九轴传感器的数据，同时再将这三个传感器中的数据进行一（姿）波（态）骚（解）操（算）作，我们就可以得到机体的欧拉角了。大家可能看到这里会有点糊涂，会想问，什么是姿态解算，什么是欧拉角。不要慌，问题不大。以下就是详细介绍：

> AHRS是自动航向基准系统(Automatic Heading Reference System)的简称。目前，使用四元数来进行AHRS姿态解算的算法被广泛采用于四轴飞行器上。该算法源自英国Bristol大学的Ph.DSebastian Madgwick，他在2009年开发并发布了该算法。下面我们来对该算法的代码进行详细分析。
> 
> 我们首先来看IMU部分。IMU是惯性测量装置(InertialMeasurement Unit)的简称，通常包含陀螺仪和加速度计。陀螺仪测量的是角速度，即物体转动的速度，把速度和时间相乘，即可以得到某一时间段内物体转过的角度。加速度计测量的是物体的加速度，我们知道，重力加速度是一个物体受重力作用的情况下所具有的加速度。当物体处于静止状态时，加速度计测量出来的值就等于重力加速度1g,约等于9.8米每平方秒。重力加速度g的方向总是竖直向下的，通过获得重力加速度在其X轴，Y轴上的分量，我们可以计算出物体相对于水平面的倾斜角度。典型的IMU惯性测量芯片为MPU6050，它被广泛采用在四轴飞行器上。


   
    