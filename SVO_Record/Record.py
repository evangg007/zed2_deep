import sys
import time
import pyzed.sl as sl
import numpy as np



def svo_record():
    zed = sl.Camera()
    FPS = 300
    fps = 0
    # 设置相机的分辨率1080和采集帧率30fps
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.camera_fps = 30  # fps可选：15、30、60、100
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    
    err = zed.open(init_params)  # 根据自定义参数打开相机
    if err != sl.ERROR_CODE.SUCCESS:
          print('open error')
          exit(1)
   # runtime_parameters = sl.RuntimeParameters()  # 设置相机获取参数
  #  runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD
    timesp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
    out_path = 'record/' + str(timesp.get_milliseconds()) + '.svo'
    recording_param = sl.RecordingParameters(out_path, sl.SVO_COMPRESSION_MODE.H264)
    record = zed.enable_recording(recording_param)

    while fps < FPS:
        zed.grab()
        print('fps:',fps)
        fps+=1

    zed.disable_recording()

if __name__ =='__main__':
    svo_record()

