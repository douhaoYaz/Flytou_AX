sample_skel

1）功能说明：
skel文件夹下面的代码, 是爱芯SDK包提供的示例参考代码, 方便客户快速的理解SKEL整个模块的配置流程.

2）使用示例：
usage: ./sample_skel <options> ...
options:
-i,     Input File(jpg/yuv)
-r,     Input File Resolution(wxh)(yuv: should be input, jpg: allow empty)
-w,     Write result image to new jpg file(path name, default: empty, not write result image)
-o,     Save ouput result(path name)
-m,     Models deployment path(path name)
-t,     Repeat times((unsigned int), default=1)
-I,     Interval sending time per frame((unsigned int)ms, default=0)
-c,     Confidence(Body)((float: 0-1), default=0)
-H,     Human track size limit((unsigned int), default=3)
-V,     Vehicle track size limit((unsigned int), default=0)
-C,     Cylcle track size limit((unsigned int), default=0)
-j,     Push jenc buffer size((unsigned int), default=0)
-d,     Cache list depth((unsigned int), default=1)
-f,     Src framerate for yuv video((unsigned int), default=25)
-F,     Dst framerate for yuv video((unsigned int), default=25)
-P,     Push face pitch filter((unsigned int(0-180)), default=180)
-Y,     Push face yaw filter((unsigned int(0-180)), default=180)
-R,     Push face roll filter((unsigned int(0-180), default=180)
-B,     Push face blur filter((float: 0-1), default=1)
-U,     Push panorama enable((unsigned int), default=0)
-T,     Push strategy interval time((unsigned int)ms, default=2000)
-N,     Push strategy counts((unsigned int), default=1)
-S,     Push strategy same frame((unsigned int), default=0(cross frame)
-u,     Skel push strategy((unsigned int), default=3)
                1: fast push strategy
                2: push strategy
                3: best push strategy
-p,     Skel PPL((unsigned int), default=1)
                1: AX_SKEL_PPL_BODY
                2: AX_SKEL_PPL_POSE
                3: AX_SKEL_PPL_FH
                4: AX_SKEL_PPL_HVCFP
                5: AX_SKEL_PPL_FACE_FEATURE
                6: AX_SKEL_PPL_HVCP
-v,     Log level((unsigned int), default=5)
                0: LOG_EMERGENCY_LEVEL
                1: LOG_ALERT_LEVEL
                2: LOG_CRITICAL_LEVEL
                3: LOG_ERROR_LEVEL
                4: LOG_WARN_LEVEL
                5: LOG_NOTICE_LEVEL
                6: LOG_INFO_LEVEL
                7: LOG_DEBUG_LEVEL
-h,     print this message

-i 表示输入的YUV数据的文件名<必须>
-r 表示输入的YUV数据的分辨率,格式: wxh，比如1280x720<如果是输入yuv时必须输入分辨率，如果输入是jpg时可以不输入>
-w 表示结果显示是否写到一个新的jpg文件；参数为需要保存的jpg文件的路径；如果写，文件将是原文件名+_result_xx.jpg[默认是不写新文件]
-o 表示将输出结果保存到指定文件夹[默认不保存]
-m 表示模型文件所在路径[不输入时，PPL(1-2):/opt/etc/skelModels, PPL(3-5):/opt/etc/Models]
-t 表示重复执行次数[默认: 1次]
-I 表示每次送帧的时间间隔[默认: 0]
-u 表示推图策略[默认: 3，最优推图]
-T 表示推图间隔[默认: 2000ms，仅最快和间隔推图有效]
-N 表示推图次数[默认: 1，仅最快和间隔推图有效]
-S 表示是否同帧推图[默认: 0，跨帧推图]
-U 表示是否推全景图[默认: 0]
-c 表示人形置信度
-H 表示检测人形数目限制[默认: 3]
-V 表示检测机动车数目限制[默认: 0]
-C 表示检测非机动车数目限制[默认: 0]
-j 表示推图所需的JENC Buffer大小的配置
-d 表示算法缓存帧大小[默认: 1]
-f 表示yuv视频源的帧率[默认: 25]
-F 表示yuv视频源的目标算法帧率[默认: 25]
-P 表示推图人脸俯仰角角度过滤[0-180，默认: 180]
-Y 表示推图人脸偏航角角度过滤[0-180，默认: 180]
-R 表示推图人脸翻滚角角度过滤[0-180，默认: 180]
-B 表示推图人脸模糊度过滤[0-1(浮点)，默认: 1]
-p 表示选择算法[默认: AX_SKEL_PPL_BODY]
-v 表示选择LOG级别[默认: LOG_NOTICE_LEVEL]
-h 打印帮助信息

注：
    默认模型位置在: /opt/etc/skelModels下
        人形模型: /opt/etc/skelModels/ax_ax620a_pose_algo_model_Vx.y.z.joint
        骨骼模型: /opt/etc/skelModels/ax_ax620a_person_algo_model_Vx.y.z.joint

3）测试结果范例
输入是YUV文件：
/opt/bin # ./sample_skel -i /opt/data/skel/1280x720_1_body.yuv  -r 1280x720 -p 2 -v 6
SKEL sample: V0.43.0 build: Aug 18 2022 18:06:23
[AX_SYS_LOG] AX_SYS_Log2ConsoleThread_Start
[I][                            main][ 960]: SKEL version: AX620_SKEL_V1.0.3
[I][                            main][ 984]: SKEL capability[0]: (ePPL: 1, PPLConfigKey: ax_ax620a_person_algo_model:V1.0.3:skel_body_ppl)
[I][                            main][ 984]: SKEL capability[1]: (ePPL: 2, PPLConfigKey: ax_ax620a_pose_algo_model:V1.0.4:skel_pose_ppl)
[I][                ParseConfigParam][ 111]: SKEL get body_max_target_count: 3
[I][                ParseConfigParam][ 121]: SKEL get vehicle_max_target_count: 0
[I][                ParseConfigParam][ 131]: SKEL get cycle_max_target_count: 0
[I][                ParseConfigParam][ 141]: SKEL get body_confidence: 0.500000
[I][                ParseConfigParam][ 151]: SKEL get face_confidence: 0.000000
[I][                ParseConfigParam][ 161]: SKEL get vehicle_confidence: 0.000000
[I][                ParseConfigParam][ 171]: SKEL get cycle_confidence: 0.000000
[I][                ParseConfigParam][ 181]: SKEL get plate_confidence: 0.000000
[I][                ParseConfigParam][ 191]: SKEL get crop_encoder_qpLevel: 90.000000
[I][                ParseConfigParam][ 201]: SKEL get body_min_size 0x0
[I][                ParseConfigParam][ 211]: SKEL get face_min_size 0x0
[I][                ParseConfigParam][ 221]: SKEL get vehicle_min_size 0x0
[I][                ParseConfigParam][ 231]: SKEL get cycle_min_size 0x0
[I][                ParseConfigParam][ 241]: SKEL get plate_min_size 0x0
[I][                ParseConfigParam][ 251]: SKEL get detect_roi [0]:[0.000000,0.000000,0.000000,0.000000]
[I][                ParseConfigParam][ 261]: SKEL get push_strategy [mode:3, times:2000, count:1, same:1]
[I][                ParseConfigParam][ 272]: SKEL get crop_encoder [0.000000, 0.000000, 0.000000, 0.000000]
[I][                ParseConfigParam][ 281]: SKEL get resize_panorama_encoder_config [0.000000, 0.000000]
[I][                ParseConfigParam][ 291]: SKEL get push_panorama [Enable: 0, Quality: 0]
[I][                ParseConfigParam][ 300]: SKEL get push_quality_body [Q: 0.000000]
[I][                ParseConfigParam][ 309]: SKEL get push_quality_vehicle [Q: 0.000000]
[I][                ParseConfigParam][ 318]: SKEL get push_quality_cycle [Q: 0.000000]
[I][                ParseConfigParam][ 330]: SKEL get push_quality_face [W: 0, H: 0, P: 180.000000, Y: 180.000000, R: 180.000000, B: 1.000000]
[I][                ParseConfigParam][ 339]: SKEL get push_quality_plate [Q: 0.000000]
[N][                            main][1273]: Task infomation:
[N][                            main][1274]:    Input file: /opt/data/skel/1280x720_1_body.yuv
[N][                            main][1275]:    Input file resolution: 1280x720
[N][                            main][1276]:    Repeat times: 1
[N][                            main][1277]: SKEL Init Elapse:
[N][                            main][1278]:    AX_SKEL_Init: 0 ms
[N][                            main][1279]:    AX_SKEL_Create: 2273 ms
[N][                            main][1305]: SKEL Process times: 1
[N][                            main][1326]: SKEL Process Elapse:
[N][                            main][1327]:    AX_SKEL_SendFrame: 3 ms
[N][                            main][1341]:    AX_SKEL_GetResult: 25 ms
[N][                            main][1343]: SKEL Process Result:
[I][                            main][1345]:    FrameId: 1
[I][                            main][1346]:    nOriginal WxH: 1280x720
[N][                            main][1348]:    Object Num: 1
[I][                            main][1364]:            FrameId: 1
[I][                            main][1365]:            TrackId: 0
[N][                            main][1370]:            Rect[0] body: [567.093811, 71.607956, 431.383057, 568.370300], Confidence: 0.920716
[N][                            main][1403]:            [0]Point Set Size: 17
[I][                            main][1411]:                    Point[0] pose: [737.687500, 132.003906] Confidence: 0.959849
[I][                            main][1411]:                    Point[1] pose: [748.921875, 120.769531] Confidence: 0.980425
[I][                            main][1411]:                    Point[2] pose: [726.453125, 120.769531] Confidence: 0.947577
[I][                            main][1411]:                    Point[3] pose: [771.390625, 132.003906] Confidence: 0.965384
[I][                            main][1411]:                    Point[4] pose: [715.218750, 132.003906] Confidence: 0.888623
[I][                            main][1411]:                    Point[5] pose: [805.093750, 188.175781] Confidence: 0.748365
[I][                            main][1411]:                    Point[6] pose: [726.453125, 210.644531] Confidence: 0.829531
[I][                            main][1411]:                    Point[7] pose: [883.734375, 255.582031] Confidence: 0.534108
[I][                            main][1411]:                    Point[8] pose: [681.515625, 278.050781] Confidence: 0.871927
[I][                            main][1411]:                    Point[9] pose: [883.734375, 278.050781] Confidence: 0.501874
[I][                            main][1411]:                    Point[10] pose: [614.109375, 300.519531] Confidence: 0.881024
[I][                            main][1411]:                    Point[11] pose: [861.265625, 356.691406] Confidence: 0.775469
[I][                            main][1411]:                    Point[12] pose: [805.093750, 367.925781] Confidence: 0.733202
[I][                            main][1411]:                    Point[13] pose: [861.265625, 480.269531] Confidence: 0.836214
[I][                            main][1411]:                    Point[14] pose: [737.687500, 457.800781] Confidence: 0.878281
[I][                            main][1411]:                    Point[15] pose: [939.906250, 603.847656] Confidence: 0.749819
[I][                            main][1411]:                    Point[16] pose: [805.093750, 581.378906] Confidence: 0.845266
[I][                            main][1428]:            Feature Size: 0
[I][                            main][1443]:    nCacheListSize: 0


输入是JPG文件：
/opt/bin # ./sample_skel -i /opt/data/skel/1280x720_1_body.jpg -p 2 -v 6
SKEL sample: V0.43.0 build: Aug 18 2022 18:06:23
[AX_SYS_LOG] AX_SYS_Log2ConsoleThread_Start
[I][                            main][ 960]: SKEL version: AX620_SKEL_V1.0.3
[I][                            main][ 984]: SKEL capability[0]: (ePPL: 1, PPLConfigKey: ax_ax620a_person_algo_model:V1.0.3:skel_body_ppl)
[I][                            main][ 984]: SKEL capability[1]: (ePPL: 2, PPLConfigKey: ax_ax620a_pose_algo_model:V1.0.4:skel_pose_ppl)
[I][                ParseConfigParam][ 111]: SKEL get body_max_target_count: 3
[I][                ParseConfigParam][ 121]: SKEL get vehicle_max_target_count: 0
[I][                ParseConfigParam][ 131]: SKEL get cycle_max_target_count: 0
[I][                ParseConfigParam][ 141]: SKEL get body_confidence: 0.500000
[I][                ParseConfigParam][ 151]: SKEL get face_confidence: 0.000000
[I][                ParseConfigParam][ 161]: SKEL get vehicle_confidence: 0.000000
[I][                ParseConfigParam][ 171]: SKEL get cycle_confidence: 0.000000
[I][                ParseConfigParam][ 181]: SKEL get plate_confidence: 0.000000
[I][                ParseConfigParam][ 191]: SKEL get crop_encoder_qpLevel: 90.000000
[I][                ParseConfigParam][ 201]: SKEL get body_min_size 0x0
[I][                ParseConfigParam][ 211]: SKEL get face_min_size 0x0
[I][                ParseConfigParam][ 221]: SKEL get vehicle_min_size 0x0
[I][                ParseConfigParam][ 231]: SKEL get cycle_min_size 0x0
[I][                ParseConfigParam][ 241]: SKEL get plate_min_size 0x0
[I][                ParseConfigParam][ 251]: SKEL get detect_roi [0]:[0.000000,0.000000,0.000000,0.000000]
[I][                ParseConfigParam][ 261]: SKEL get push_strategy [mode:3, times:2000, count:1, same:1]
[I][                ParseConfigParam][ 272]: SKEL get crop_encoder [0.000000, 0.000000, 0.000000, 0.000000]
[I][                ParseConfigParam][ 281]: SKEL get resize_panorama_encoder_config [0.000000, 0.000000]
[I][                ParseConfigParam][ 291]: SKEL get push_panorama [Enable: 0, Quality: 0]
[I][                ParseConfigParam][ 300]: SKEL get push_quality_body [Q: 0.000000]
[I][                ParseConfigParam][ 309]: SKEL get push_quality_vehicle [Q: 0.000000]
[I][                ParseConfigParam][ 318]: SKEL get push_quality_cycle [Q: 0.000000]
[I][                ParseConfigParam][ 330]: SKEL get push_quality_face [W: 0, H: 0, P: 180.000000, Y: 180.000000, R: 180.000000, B: 1.000000]
[I][                ParseConfigParam][ 339]: SKEL get push_quality_plate [Q: 0.000000]
[N][                            main][1273]: Task infomation:
[N][                            main][1274]:    Input file: /opt/data/skel/1280x720_1_body.jpg
[N][                            main][1275]:    Input file resolution: 1280x720
[N][                            main][1276]:    Repeat times: 1
[N][                            main][1277]: SKEL Init Elapse:
[N][                            main][1278]:    AX_SKEL_Init: 0 ms
[N][                            main][1279]:    AX_SKEL_Create: 2274 ms
[N][                            main][1305]: SKEL Process times: 1
[N][                            main][1326]: SKEL Process Elapse:
[N][                            main][1327]:    AX_SKEL_SendFrame: 3 ms
[N][                            main][1341]:    AX_SKEL_GetResult: 25 ms
[N][                            main][1343]: SKEL Process Result:
[I][                            main][1345]:    FrameId: 1
[I][                            main][1346]:    nOriginal WxH: 1280x720
[N][                            main][1348]:    Object Num: 1
[I][                            main][1364]:            FrameId: 1
[I][                            main][1365]:            TrackId: 0
[N][                            main][1370]:            Rect[0] body: [567.093811, 71.607956, 431.383057, 568.370300], Confidence: 0.920716
[N][                            main][1403]:            [0]Point Set Size: 17
[I][                            main][1411]:                    Point[0] pose: [737.687500, 132.003906] Confidence: 0.959849
[I][                            main][1411]:                    Point[1] pose: [748.921875, 120.769531] Confidence: 0.980425
[I][                            main][1411]:                    Point[2] pose: [726.453125, 120.769531] Confidence: 0.947577
[I][                            main][1411]:                    Point[3] pose: [771.390625, 132.003906] Confidence: 0.965384
[I][                            main][1411]:                    Point[4] pose: [715.218750, 132.003906] Confidence: 0.888623
[I][                            main][1411]:                    Point[5] pose: [805.093750, 188.175781] Confidence: 0.748365
[I][                            main][1411]:                    Point[6] pose: [726.453125, 210.644531] Confidence: 0.829531
[I][                            main][1411]:                    Point[7] pose: [883.734375, 255.582031] Confidence: 0.534108
[I][                            main][1411]:                    Point[8] pose: [681.515625, 278.050781] Confidence: 0.871927
[I][                            main][1411]:                    Point[9] pose: [883.734375, 278.050781] Confidence: 0.501874
[I][                            main][1411]:                    Point[10] pose: [614.109375, 300.519531] Confidence: 0.881024
[I][                            main][1411]:                    Point[11] pose: [861.265625, 356.691406] Confidence: 0.775469
[I][                            main][1411]:                    Point[12] pose: [805.093750, 367.925781] Confidence: 0.733202
[I][                            main][1411]:                    Point[13] pose: [861.265625, 480.269531] Confidence: 0.836214
[I][                            main][1411]:                    Point[14] pose: [737.687500, 457.800781] Confidence: 0.878281
[I][                            main][1411]:                    Point[15] pose: [939.906250, 603.847656] Confidence: 0.749819
[I][                            main][1411]:                    Point[16] pose: [805.093750, 581.378906] Confidence: 0.845266
[I][                            main][1428]:            Feature Size: 0
[I][                            main][1443]:    nCacheListSize: 0
：
