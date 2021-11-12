# MPU6050

這是一個練習用 STM32 透過 MPU6050 姿態感測器來取得三軸上的加速度數據與圍繞三軸旋轉的角速度數據，在經過數據融合演算法 ( 卡爾曼濾波器 ) 來計算更加穩定可靠的姿態角度。

此程式僅使用到 MPU6050 的最基本功能，加速度計與陀螺儀。

* 硬體

  STM32F446RE Nucleo-64

  MPU6050 姿態感測器

* 未來

  嘗試使用不同的數據融合演算法。