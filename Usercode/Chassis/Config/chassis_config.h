/*** 
 * @Author: OriTwil
 * @Description: 用户配置文件
 * @FilePath: \ER\Usercode\user_inc\useful_constant.h
 * @@WeChat:szf13373959031
 */

/**
 * @brief 一些有用的常量，基本上是从 GCC 的 math.h 中复制下来的(因为 ARMCC 的库里貌似没有)
 * 
 */

#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__

/* Natural log of 2 */
#define _M_LN2        0.693147180559945309417

#define MAXFLOAT	3.40282347e+38F

#define M_E		2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		_M_LN2
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846
#define DEC          (M_PI / 180)

#ifndef M_PI_2
#define M_PI_2		1.57079632679489661923
#endif

#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440

#define M_TWOPI         (M_PI * 2.0)
#define M_3PI_4		2.3561944901923448370E0
#define M_SQRTPI        1.77245385090551602792981
#define M_LN2LO         1.9082149292705877000E-10
#define M_LN2HI         6.9314718036912381649E-1
#define M_SQRT3	1.73205080756887719000
#define M_IVLN10        0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E        _M_LN2
#define M_INVLN2        1.4426950408889633870E0  /* 1 / log(2) */

// 限幅
#define max(a, b)      (a > b ? a : b)
#define min(a, b)      (a < b ? a : b)
#define range(x, a, b) (min(max(x, a), b))

/**
 * @brief 端口
 * 
 */
#define huart_AS69 huart1
#define huart_OPS huart7
#define huart_Computer huart3
#define huart_Remote_Control huart8
#define huart_Chassis_to_Upper huart6
#define UART_Remote_Control UART8
#define UART_Computer USART3
#define UART_OPS UART7
#define UART_AS69 USART1
/**
 * @brief id
 * 
 */

/**
 * @brief parameters
 * 
 */

// 底盘
#define rotate_ratio 0.3615 // (Width + Length)/2
#define wheel_rpm_ratio 2387.324 // 换算线速度到rpm

// 挡板
#define BAFFLE_UP -60

// 偏航角度
#define w_5_1 19.6041
#define w_5_2 -19.6041
#define w_5_3 0
#define w_5_4 51.0725
#define w_5_5 -51.0725
#define w_5_6 105.7086
#define w_5_7 180
#define w_5_8 -105.7086

#define w_6_1 0
#define w_6_4 0
#define w_6_6 0

#define w_7_2 0
#define w_7_5 0
#define w_7_8 0

#endif
