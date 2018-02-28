//
// Created by eric on 2017/11/1.
//
#include <jni.h>
#include <string>
#include<android/log.h>
#include <exception>

//定义日志宏变量
#define logw(content)   __android_log_write(ANDROID_LOG_WARN,"eric",content)
#define loge(content)   __android_log_write(ANDROID_LOG_ERROR,"eric",content)
#define logd(content)   __android_log_write(ANDROID_LOG_DEBUG,"eric",content)

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
//引入时间
#include "libavutil/time.h"
#include "libavutil/imgutils.h"
}

#include <iostream>

using namespace std;

jobject pushCallback = NULL;
jclass cls = NULL;
jmethodID mid = NULL;


int avError(int errNum) {
    char buf[1024];
    //获取错误信息
    av_strerror(errNum, buf, sizeof(buf));
    loge(string().append("发生异常：").append(buf).c_str());
    return -1;
}




//=======================================================================
//
// 摄像头采集数据并输出（文件/RTMP推流）
//
//=======================================================================


AVFormatContext *ofmt_ctx;
AVStream *video_st;
AVCodecContext *pCodecCtx;
AVCodec *pCodec;
AVPacket enc_pkt;
AVFrame *pFrameYUV;
int count = 0;
int yuv_width;
int yuv_height;
int y_length;
int uv_length;
int width = 320;
int height = 240;
int fps = 15;

/**
 * 初始化
 */
extern "C"
JNIEXPORT jint JNICALL
Java_com_wangheart_rtmpfile_ffmpeg_FFmpegHandle_initVideo(JNIEnv *env, jobject instance,
                                                          jstring url_) {
    const char *out_path = env->GetStringUTFChars(url_, 0);
    logd(out_path);

    //计算yuv数据的长度
    yuv_width = width;
    yuv_height = height;
    y_length = width * height;
    uv_length = width * height / 4;

    av_register_all();

    //output initialize
    avformat_alloc_output_context2(&ofmt_ctx, NULL, "flv", out_path);
    //output encoder initialize
    pCodec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!pCodec) {
        loge("Can not find encoder!\n");
        return -1;
    }
    pCodecCtx = avcodec_alloc_context3(pCodec);
    //编码器的ID号，这里为264编码器，可以根据video_st里的codecID 参数赋值
    pCodecCtx->codec_id = pCodec->id;
    //像素的格式，也就是说采用什么样的色彩空间来表明一个像素点
    pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    //编码器编码的数据类型
    pCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    //编码目标的视频帧大小，以像素为单位
    pCodecCtx->width = width;
    pCodecCtx->height = height;
    pCodecCtx->framerate = (AVRational) {fps, 1};
    //帧率的基本单位，我们用分数来表示，
    pCodecCtx->time_base = (AVRational) {1, fps};
    //目标的码率，即采样的码率；显然，采样码率越大，视频大小越大
    pCodecCtx->bit_rate = 400000;
    //固定允许的码率误差，数值越大，视频越小
//    pCodecCtx->bit_rate_tolerance = 4000000;
    pCodecCtx->gop_size = 50;
    /* Some formats want stream headers to be separate. */
    if (ofmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        pCodecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;

    //H264 codec param
//    pCodecCtx->me_range = 16;
    //pCodecCtx->max_qdiff = 4;
    pCodecCtx->qcompress = 0.6;
    //最大和最小量化系数
    pCodecCtx->qmin = 10;
    pCodecCtx->qmax = 51;
    //Optional Param
    //两个非B帧之间允许出现多少个B帧数
    //设置0表示不使用B帧
    //b 帧越多，图片越小
    pCodecCtx->max_b_frames = 0;
    // Set H264 preset and tune
    AVDictionary *param = 0;
    //H.264
    if (pCodecCtx->codec_id == AV_CODEC_ID_H264) {
//        av_dict_set(&param, "preset", "slow", 0);
        /**
         * 这个非常重要，如果不设置延时非常的大
         * ultrafast,superfast, veryfast, faster, fast, medium
         * slow, slower, veryslow, placebo.　这是x264编码速度的选项
       */
        av_dict_set(&param, "preset", "superfast", 0);
        av_dict_set(&param, "tune", "zerolatency", 0);
    }

    if (avcodec_open2(pCodecCtx, pCodec, &param) < 0) {
        loge("Failed to open encoder!\n");
        return -1;
    }

    //Add a new stream to output,should be called by the user before avformat_write_header() for muxing
    video_st = avformat_new_stream(ofmt_ctx, pCodec);
    if (video_st == NULL) {
        return -1;
    }
    video_st->time_base.num = 1;
    video_st->time_base.den = fps;
//    video_st->codec = pCodecCtx;
    video_st->codecpar->codec_tag = 0;
    avcodec_parameters_from_context(video_st->codecpar, pCodecCtx);

    //Open output URL,set before avformat_write_header() for muxing
    if (avio_open(&ofmt_ctx->pb, out_path, AVIO_FLAG_READ_WRITE) < 0) {
        loge("Failed to open output file!\n");
        return -1;
    }

    //Write File Header
    avformat_write_header(ofmt_ctx, NULL);

    return 0;
}

/**
 * H264编码并输出
 */
int64_t startTime = 0;
extern "C"
JNIEXPORT jint JNICALL
Java_com_wangheart_rtmpfile_ffmpeg_FFmpegHandle_onFrameCallback(JNIEnv *env, jobject instance,
                                                                jbyteArray buffer_) {
//    startTime = av_gettime();
    jbyte *in = env->GetByteArrayElements(buffer_, NULL);

    int ret = 0;

    pFrameYUV = av_frame_alloc();
    int picture_size = av_image_get_buffer_size(pCodecCtx->pix_fmt, pCodecCtx->width,
                                                pCodecCtx->height, 1);
    uint8_t *buffers = (uint8_t *) av_malloc(picture_size);


    //将buffers的地址赋给AVFrame中的图像数据，根据像素格式判断有几个数据指针
    av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, buffers, pCodecCtx->pix_fmt,
                         pCodecCtx->width, pCodecCtx->height, 1);

    //安卓摄像头数据为NV21格式，此处将其转换为YUV420P格式
    ////N21   0~width * height是Y分量，  width*height~ width*height*3/2是VU交替存储
    //复制Y分量的数据
    memcpy(pFrameYUV->data[0], in, y_length); //Y
    pFrameYUV->pts = count;
    for (int i = 0; i < uv_length; i++) {
        //将v数据存到第三个平面
        *(pFrameYUV->data[2] + i) = *(in + y_length + i * 2);
        //将U数据存到第二个平面
        *(pFrameYUV->data[1] + i) = *(in + y_length + i * 2 + 1);
    }

    pFrameYUV->format = AV_PIX_FMT_YUV420P;
    pFrameYUV->width = yuv_width;
    pFrameYUV->height = yuv_height;

    //例如对于H.264来说。1个AVPacket的data通常对应一个NAL
    //初始化AVPacket
    av_init_packet(&enc_pkt);
//    __android_log_print(ANDROID_LOG_WARN, "eric", "编码前时间:%lld",
//                        (long long) ((av_gettime() - startTime) / 1000));
    //开始编码YUV数据
    ret = avcodec_send_frame(pCodecCtx, pFrameYUV);
    if (ret != 0) {
        logw("avcodec_send_frame error");
        return -1;
    }
    //获取编码后的数据
    ret = avcodec_receive_packet(pCodecCtx, &enc_pkt);
//    __android_log_print(ANDROID_LOG_WARN, "eric", "编码时间:%lld",
//                        (long long) ((av_gettime() - startTime) / 1000));
    //是否编码前的YUV数据
    av_frame_free(&pFrameYUV);
    if (ret != 0 || enc_pkt.size <= 0) {
        loge("avcodec_receive_packet error");
        avError(ret);
        return -2;
    }
    enc_pkt.stream_index = video_st->index;
    AVRational time_base = ofmt_ctx->streams[0]->time_base;//{ 1, 1000 };
    enc_pkt.pts = count * (video_st->time_base.den) / ((video_st->time_base.num) * fps);
    enc_pkt.dts = enc_pkt.pts;
    enc_pkt.duration = (video_st->time_base.den) / ((video_st->time_base.num) * fps);
    __android_log_print(ANDROID_LOG_WARN, "eric",
                        "index:%d,pts:%lld,dts:%lld,duration:%lld,time_base:%d,%d",
                        count,
                        (long long) enc_pkt.pts,
                        (long long) enc_pkt.dts,
                        (long long) enc_pkt.duration,
                        time_base.num, time_base.den);
    enc_pkt.pos = -1;


    ret = av_interleaved_write_frame(ofmt_ctx, &enc_pkt);
    if (ret != 0) {
        loge("av_interleaved_write_frame failed");
    }
    count++;
    env->ReleaseByteArrayElements(buffer_, in, 0);
    return 0;

}

/**
 * 释放资源
 */
extern "C"
JNIEXPORT jint JNICALL
Java_com_wangheart_rtmpfile_ffmpeg_FFmpegHandle_close(JNIEnv *env, jobject instance) {
    if (video_st)
        avcodec_close(video_st->codec);
    if (ofmt_ctx) {
        avio_close(ofmt_ctx->pb);
        avformat_free_context(ofmt_ctx);
        ofmt_ctx = NULL;
    }
    return 0;
}