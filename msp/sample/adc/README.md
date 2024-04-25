1.本测试用来验证获取adc通道数据读取功能

2.运行方法
eg： 使能通道0到通道4，bit0代表使能通道0，bit0代表使能通道1，以此类推，总共最大支持5个通道；
#./sample_adc 0x1f


3.正常运行log输出结果如下：

                 ADC Driver Test Example.

Current ADC date is chan0:0x83, chan1:0xf7, chan2:0xdf， chan3:0xdd, chan4:0xe3