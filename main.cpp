#include <iostream>
#include <queue>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp> //视频数据显示
#include <alsa/asoundlib.h> //音频数据播放


#ifdef __cplusplus
extern "C"{
#endif

#include <libavformat/avio.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>

#ifdef __cplusplus
}
#endif


cv::Mat avframe_to_cvmat(AVFrame *frame)
{
        AVFrame dst;
        cv::Mat m;
        memset(&dst, 0, sizeof(dst));
        int w = frame->width, h = frame->height;
        m = cv::Mat(h, w, CV_8UC3);
        dst.data[0] = (uint8_t *)m.data;
        avpicture_fill( (AVPicture *)&dst, dst.data[0], AV_PIX_FMT_BGR24, w, h);
        struct SwsContext *convert_ctx=NULL;
        AVPixelFormat src_pixfmt = (AVPixelFormat)frame->format;
        AVPixelFormat dst_pixfmt = AV_PIX_FMT_BGR24;
        convert_ctx = sws_getContext(w, h, src_pixfmt, w, h, dst_pixfmt,SWS_FAST_BILINEAR, NULL, NULL, NULL);
        sws_scale(convert_ctx, frame->data, frame->linesize, 0, h,dst.data, dst.linesize);
        sws_freeContext(convert_ctx);
        return m;
}

//音视频频播放线程
std::mutex mtx_video_play;
std::condition_variable cond_not_empty_video_play;
std::condition_variable cond_not_full_video_play;
const unsigned int QUEUE_VIDEO_PLAY_MAX_SIZE = 100;
std::queue<AVFrame> queue_video_play;//视频帧数据播放队列


std::mutex mtx_audio_play;
std::condition_variable cond_not_empty_audio_play;
std::condition_variable cond_not_full_audio_play;
const unsigned int QUEUE_AUDIO_PLAY_MAX_SIZE = 100;
std::queue<AVFrame> queue_audio_play;//音频帧数据播放队列




std::mutex mtx_video_decode;
std::condition_variable cond_not_empty_video_decode;
std::condition_variable cond_not_full_video_decode;
const unsigned int QUEUE_VIDEO_DECODE_MAX_SIZE = 100;
std::queue<AVPacket> queue_video_decode;

bool video_decode_thread_interrupt = false;
void video_decode(AVStream *videoStream,
                  AVCodecContext* videoCodecContext,
                  AVCodec* videoCodec)
{
    AVFrame *videoFrame = av_frame_alloc();//分配视频帧
    while ( ! video_decode_thread_interrupt)
    {
        std::unique_lock<std::mutex> lock(mtx_video_decode);
        if(queue_video_decode.empty()){
            cond_not_empty_video_decode.wait(lock);//等待解码队列数据不为空
        }

        AVPacket avPacket = queue_video_decode.front();//取出一帧视频帧
        queue_video_decode.pop();//出队

        int got_picture_ptr = 0;//是否被解码成功解码的标识(0标识失败,非0表示成功)
        avcodec_decode_video2(videoCodecContext,videoFrame,&got_picture_ptr,&avPacket);//解码视频帧
        if(0 != got_picture_ptr)
        {
            std::cout<<"视频解码成功format:"<<videoFrame->format<<std::endl;

            std::unique_lock<std::mutex> lock(mtx_video_play);
            if(queue_video_play.size() >= QUEUE_VIDEO_PLAY_MAX_SIZE){
                std::cout << "视频播放队列已满" << std::endl;
                cond_not_full_video_play.wait(lock); //等待播放队列不满
            }

            AVFrame *temp = av_frame_alloc();
            if(av_frame_ref(temp,videoFrame) >= 0){
                queue_video_play.push(*temp);
            }

            cond_not_empty_video_play.notify_all();
        }

        av_packet_unref(&avPacket);//取消对应数据区的引用
        cond_not_full_video_decode.notify_all();
    }
    av_frame_free(&videoFrame);//释放videoFrame
}

std::mutex mtx_audio_decode;
std::condition_variable cond_not_empty_audio_decode;
std::condition_variable cond_not_full_audio_decode;
const unsigned int QUEUE_AUDIO_DECODE_MAX_SIZE = 100;
std::queue<AVPacket> queue_audio_decode;

bool audio_decode_thread_interrupt = false;
void audio_decode(AVStream *audioStream,
                  AVCodecContext* audioCodecContext,
                  AVCodec* audioCodec)
{
    AVFrame *audioFrame = av_frame_alloc();//分配音频帧
    while ( ! audio_decode_thread_interrupt)
    {
        std::unique_lock<std::mutex> lock(mtx_audio_decode);
        if(queue_audio_decode.empty()){
            cond_not_empty_audio_decode.wait(lock);
        }

        AVPacket avPacket = queue_audio_decode.front();//取出一帧视频帧
        queue_audio_decode.pop();//出队


        int got_frame_ptr = 0;//是否被解码成功的标识
        avcodec_decode_audio4(audioCodecContext,audioFrame,&got_frame_ptr,&avPacket);//解码音频帧
        if(0 != got_frame_ptr)
        {
            std::cout<<"音频解码成功format:"<<audioFrame->format<<std::endl;

            std::unique_lock<std::mutex> lock(mtx_audio_play);
            if(queue_audio_play.size() >= QUEUE_AUDIO_PLAY_MAX_SIZE){
                std::cout<<"音频播放队列已满"<<std::endl;
                cond_not_full_audio_play.wait(lock);//等待播放队列不满
            }

            AVFrame *temp = av_frame_alloc();
            if(av_frame_ref(temp,audioFrame) >= 0){
                queue_audio_play.push(*temp);
            }

            cond_not_empty_audio_play.notify_all();
        }

        av_packet_unref(&avPacket);//取消对应数据区的引用
        cond_not_full_audio_decode.notify_all();
    }
    av_frame_free(&audioFrame);//释放audioFrame
}

volatile int64_t audioClock = 0;
bool audio_play_thread_interrupt = false;
void audio_play(AVStream *audioStream,
                AVCodecContext* audioCodecContext,
                AVCodec* audioCodec)
{
    /*1. 打开pcm设备 */
    snd_pcm_t *handle;
    int rc = snd_pcm_open(&handle, "default",SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        printf("open device failed\n");
        return;
    }
    /*2. 分配一个硬件参数对象 */
    snd_pcm_hw_params_t *params;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle, params);/* 使用默认值填充参数对象. */
    /*2.1 设置硬件参数 */
    snd_pcm_access_t access_mode = SND_PCM_ACCESS_RW_INTERLEAVED;//访问模式:交错访问
    snd_pcm_hw_params_set_access(handle, params,access_mode);/* 交错模式 Interleaved mode */
    /*2.2 设置音频格式*/
    snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;//采样位数:16位,小端存储
    switch (audioCodecContext->sample_fmt) {//采样格式
    case AV_SAMPLE_FMT_U8:
        format = SND_PCM_FORMAT_U8;//8位
        break;
    case AV_SAMPLE_FMT_S16:
        format = SND_PCM_FORMAT_S16_LE;//16位
        break;
    }
    snd_pcm_hw_params_set_format(handle, params,format);/* 采样位数 Signed 16-bit little-endian format */
    /*2.3 设置通道数*/
    int channel = 2;
    channel = audioCodecContext->channels;//声道数(音频)
    std::cout << "音频通道设置:" << channel << std::endl;
    snd_pcm_hw_params_set_channels(handle, params, channel);/* 通道数 Two channels (stereo) */
    /*2.4 设置采样率*/
    unsigned int simple_rate = 44100;//采样率
    simple_rate = audioCodecContext->sample_rate;
    std::cout << "音频采样率设置:" << simple_rate << std::endl;
    snd_pcm_hw_params_set_rate_near(handle, params,&simple_rate, NULL);/* 采样率 44100 bits/second sampling rate (CD quality) */
    /*2.5 设置一个周期的帧数量*/
    snd_pcm_uframes_t frames = 1024;//一个周期多少帧
    snd_pcm_hw_params_set_period_size_near(handle,params, &frames, NULL);//设置一个周期的多少帧
    //int dir;//设备采样率与输入采样的偏差
    /*2.6 将设置好的参数写入驱动 */
    rc = snd_pcm_hw_params(handle, params);
    if (rc < 0) {
        printf("unable to set hw parameters: %s\n",snd_strerror(rc));
        return;
    }
    /* 获取一个周期的大小(帧) Use a buffer large enough to hold one period */
    snd_pcm_hw_params_get_period_size(params, &frames,NULL);
    printf("frames = %ld\n",frames);


    /*对解码的数据进行重新采样*/
    SwrContext *swrCtx = swr_alloc();
    enum AVSampleFormat in_sample_fmt = audioCodecContext->sample_fmt; //输入的采样格式
    enum AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16; //输出的采样格式 16bit PCM
    int in_sample_rate = audioCodecContext->sample_rate; //输入的采样率
    int out_sample_rate = 44100; //输出的采样率
    uint64_t in_ch_layout = audioCodecContext->channel_layout; //输入的声道布局
    uint64_t out_ch_layout = AV_CH_LAYOUT_STEREO; //输出的声道布局
    swr_alloc_set_opts(swrCtx,
                       out_ch_layout,  //输出通道布局
                       out_sample_fmt, //输出采样格式
                       out_sample_rate,//输出采样率
                       in_ch_layout,   //输入采样布局
                       in_sample_fmt,  //输入采样格式
                       in_sample_rate, //输入采样率
                       0,    //logging level offset
                       NULL);//parent logging context, can be NULL
    swr_init(swrCtx);

    unsigned int out_buffer_size = 2*2*out_sample_rate;// 两通道 * 16位采样格式 * 采样率
    uint8_t *out_buffer = (uint8_t*)av_malloc(out_buffer_size);

    while( ! audio_play_thread_interrupt){

        std::unique_lock<std::mutex> lock(mtx_audio_play);
        if(queue_audio_play.empty()){
            std::cout << "音频播放队列暂停" << std::endl;
            cond_not_empty_audio_play.wait(lock);
        }

        AVFrame audioFrame = queue_audio_play.front();
        queue_audio_play.pop();
        swr_convert(swrCtx,  //转换上下文
                    &out_buffer, //输出buff
                    out_buffer_size, //输出空间大小
                    (const uint8_t**)audioFrame.data, //输入buffer
                    audioFrame.nb_samples); //采样数量

        std::cout<<"播放音频帧:"<<audioFrame.pts*av_q2d(audioCodecContext->time_base)<<std::endl;

//        audioClock = audioFrame.pts * av_q2d(audioCodecContext->time_base);

        /*将音频数据喂给设备*/
        rc = snd_pcm_writei(handle, out_buffer, audioFrame.nb_samples);//阻塞写入
        if (rc == -EPIPE){
            printf("underrun occurred\n");/* EPIPE means underrun */
            snd_pcm_prepare(handle);
        }else if (rc < 0){
            printf("error from writei: %s\n",snd_strerror(rc));
        }


        av_frame_unref(&audioFrame);
        cond_not_full_audio_play.notify_all();
    }
    av_free(out_buffer);//释放音频采样缓冲区
    swr_free(&swrCtx);//释放音频采样上下文

    //关闭PCM设备
    snd_pcm_drain(handle);
    snd_pcm_close(handle);
}


bool video_play_thread_interrupt = false;
void video_play(AVStream *videoStream,
                AVCodecContext* videoCodecContext,
                AVCodec* videoCodec,
                AVCodecContext* audioCodecContext,
                AVCodec* audioCodec)
{
    int out_frame_width = 640;
    int out_frame_height = 480;

    //创建图像格式转换上下文
    SwsContext *rgbSwsContext = sws_getContext(videoCodecContext->width,//原图像宽
                                               videoCodecContext->height,//原图像高
                                               videoCodecContext->pix_fmt,//原图像格式
                                               out_frame_width,//目标图像宽
                                               out_frame_height,//目标图像高
                                               AV_PIX_FMT_BGR24,//目标图像格式
                                               SWS_BICUBIC,//转换算法
                                               NULL,//原图像滤波
                                               NULL,//目标图像滤波
                                               NULL);//其他参数
    //图像数据缓冲区
    AVFrame* rgbFrame = av_frame_alloc();
    //图像缓冲区大小
    int rgbBuffer_size=avpicture_get_size(AV_PIX_FMT_BGR24, out_frame_width,out_frame_height);
    uint8_t* rgbBuffer=(uint8_t*)av_malloc(rgbBuffer_size*sizeof(uint8_t));
    avpicture_fill((AVPicture *)rgbFrame, rgbBuffer, AV_PIX_FMT_BGR24,out_frame_width, out_frame_height);

    int fps = 30;
    if(videoStream ->avg_frame_rate.den && videoStream ->avg_frame_rate.num){
        fps = av_q2d(videoStream->avg_frame_rate);
    }else{
        fps=30.0;
    }

    while (! video_play_thread_interrupt)
    {

        std::unique_lock<std::mutex> lock(mtx_video_play);
        if(queue_video_play.empty()){
            cond_not_empty_video_play.wait(lock);
        }

        AVFrame videoFrame = queue_video_play.front();
        queue_video_play.pop();

        sws_scale(rgbSwsContext,  //转换上下文
                  (uint8_t const * const *)videoFrame.data,  //输入图像数据
                  videoFrame.linesize, //输入图像行尺寸(宽度)
                  0, //第一列要处理的位置(从第几行数据开始处理)
                  videoFrame.height, //输入图像高度
                  rgbFrame->data,   //输出图像数据
                  rgbFrame->linesize); //输出图像行尺寸(宽度)

        std::cout<<"播放视频帧:"<<videoFrame.pts*av_q2d(videoCodecContext->time_base) << "当前帧率:    " << fps <<std::endl;


//        double last_play  //上一帧的播放时间
//         ,play             //当前帧的播放时间
//         ,last_delay    // 上一次播放视频的两帧视频间隔时间
//         ,delay         //两帧视频间隔时间
//         ,audio_clock //音频轨道 实际播放时间
//         ,diff   //音频帧与视频帧相差时间
//         ,sync_threshold
//         ,start_time  //从第一帧开始的绝对时间
//         ,pts
//         ,actual_delay;//真正需要延迟时间


//        if((pts=videoFrame.best_effort_timestamp)==AV_NOPTS_VALUE){
//            pts=0;
//        }
//        play = pts*av_q2d(videoCodecContext->time_base);

//        play = synchronize(&videoFrame,play);//纠正时间


//        delay = play - last_play;
//        if (delay <= 0 || delay > 1) {
//            delay = last_delay;
//        }
//        audio_clock = audioCodecContext->clock;
//        last_delay = delay;
//        last_play = play;
////音频与视频的时间差
//        diff = ffmpegVideo->clock - audio_clock;
////        在合理范围外  才会延迟  加快
//        sync_threshold = (delay > 0.01 ? 0.01 : delay);

//        if (fabs(diff) < 10) {
//            if (diff <= -sync_threshold) {
//                delay = 0;
//            } else if (diff >=sync_threshold) {
//                delay = 2 * delay;
//            }
//        }
//        start_time += delay;
//        actual_delay=start_time-av_gettime()/1000000.0;
//        if (actual_delay < 0.01) {
//            actual_delay = 0.01;
//        }
//        av_usleep(actual_delay*1000000.0+6000);


        cv::Mat mat = avframe_to_cvmat(&videoFrame);
        cv::imshow("picture",mat);
        cv::waitKey(1);//1ms

        av_frame_unref(&videoFrame);
        cond_not_full_video_play.notify_all();
    }

    av_free(rgbBuffer);//释放缓冲区数据
    av_frame_free(&rgbFrame);//释放RGBFrame

    sws_freeContext(rgbSwsContext);//释放图像转换上下文
}



const char *in_filename = "test.mp4";//输入URL(mov\mkv\ts\mp4\h264）
const char *out_filename = "rtmp://123.206.23.239:1935/wstv/home";//输出URL(rtmp://123.206.23.239:1935/wstv/home或rtp://233.233.233.233:6666)

bool ffmpeg_init_thread_interrupt = false;
int ffmpeg_init()
{
    //1. 首先注册ffmpeg所有的编解码器
    av_register_all();

    //2.1 打开输入文件
    AVFormatContext* avFormatContext = avformat_alloc_context();
    int ret = avformat_open_input(&avFormatContext, in_filename, NULL, NULL);//读取文件头
    if( ret != 0 ){
        printf("Couldn't open input file.\n");
        return -1;
    }

    //2.2 打印文件的详细信息
    av_dump_format(avFormatContext, -1, in_filename, 0);

    //2.3 获取流信息
    ret = avformat_find_stream_info(avFormatContext, NULL);
    if( ret < 0 ){//获取文件中的流信息
        printf("Couldn't get input file infomation.\n");
        return -1;
    }

    //3. 获取对应流的索引号
    int videoStream_index = -1;//视频流索引号
    int audioStream_index = -1;//音频流索引号
    for(int i = 0; i < avFormatContext->nb_streams; i++ ){
        if(avFormatContext->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO){
            videoStream_index = i;
        }
        if(avFormatContext->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO){
            audioStream_index = i;
        }
    }
    if( videoStream_index == -1 || audioStream_index == -1 ){
        printf("Couldn't get input file videoStream or audioStream!\n");
        return -1;
    }

    //3.1 获取视频流的编码上下文
    AVStream *videoStream = avFormatContext->streams[videoStream_index];//获取视频流
    AVCodecContext* videoCodecContext = avFormatContext->streams[videoStream_index]->codec;
    AVCodec* videoCodec = avcodec_find_decoder(videoCodecContext->codec_id);//通过上下文查找编码器
    if( videoCodec == NULL ){
        printf("Couldn't get AVCodec\n");
        return -1;
    }
    if( avcodec_open2(videoCodecContext, videoCodec, NULL) < 0 ){// 打开编码器
        printf("Couldn't open AVCodec\n");
        return -1;
    }

    //3.2 获取音频流的编解码上下文
    AVStream *audioStream = avFormatContext->streams[audioStream_index];//获取视频流
    AVCodecContext* audioCodecContext = avFormatContext->streams[audioStream_index]->codec;
    AVCodec* audioCodec = avcodec_find_decoder(audioCodecContext->codec_id);//通过上下文查找编码器
    if( audioCodec == NULL ){
        printf("Couldn't get AVCodec\n");
        return -1;
    }
    if( avcodec_open2(audioCodecContext, audioCodec, NULL) < 0 ){// 打开编码器
        printf("Couldn't open AVCodec\n");
        return -1;
    }

    std::thread thread_video_decode(video_decode,videoStream,videoCodecContext,videoCodec);
    std::thread thread_audio_decode(audio_decode,audioStream,audioCodecContext,audioCodec);
    std::thread thread_video_play(video_play,videoStream,videoCodecContext,videoCodec,audioCodecContext,audioCodec);
    std::thread thread_audio_play(audio_play,audioStream,audioCodecContext,audioCodec);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    //4.1 开始读取数据
    AVPacket *avPacket = av_packet_alloc();//分配avPacket
    while (!ffmpeg_init_thread_interrupt && av_read_frame(avFormatContext,avPacket) >= 0)
    {
        if(avPacket->stream_index == videoStream_index){

            std::unique_lock<std::mutex> lock(mtx_video_decode);
            if(queue_video_decode.size() > QUEUE_VIDEO_DECODE_MAX_SIZE){
                std::cout << "视频解码队列已满" << std::endl;
                cond_not_full_video_decode.wait(lock);//等待解码队列不满
            }

            AVPacket *pkt = av_packet_alloc();
            if (av_packet_ref(pkt, avPacket) >= 0){ //将pkt指针指向avPacket的数据区域，引用+1
                queue_video_decode.push(*pkt); //添加到视频解码队列
            }

            cond_not_empty_video_decode.notify_all();
        }
        if(avPacket->stream_index == audioStream_index){

            std::unique_lock<std::mutex> lock(mtx_audio_decode);
            if(queue_audio_decode.size() > QUEUE_AUDIO_DECODE_MAX_SIZE){
                std::cout << "音频解码队列已满" << std::endl;
                cond_not_full_audio_decode.wait(lock);
            }

            AVPacket *pkt = av_packet_alloc();
            if (av_packet_ref(pkt, avPacket) >= 0){ //将pkt指针指向avPacket的数据区域，引用+1
                queue_audio_decode.push(*pkt); //添加到音频解码队列
            }

            cond_not_empty_audio_decode.notify_all();
        }

        av_packet_unref(avPacket); // 不要忘记减少引用计数
    }
    av_packet_free(&avPacket);//释放avPacket


    std::cout << "视频读取结束" << std::endl;

    video_decode_thread_interrupt = true;
    audio_decode_thread_interrupt = true;
    video_play_thread_interrupt = true;
    audio_play_thread_interrupt = true;
    thread_video_decode.join();
    thread_audio_decode.join();
    thread_video_play.join();
    thread_audio_play.join();

    avcodec_close(videoCodecContext);//关闭音频解码器
    avcodec_close(audioCodecContext);//关闭视频解码器
    avformat_close_input(&avFormatContext);//关闭输入文件
    return 0;
}


int main()
{
    ffmpeg_init();
}

