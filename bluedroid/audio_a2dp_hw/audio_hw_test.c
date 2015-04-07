//<MStar Software>
//******************************************************************************
// MStar Software
// Copyright (c) 2010 - 2012 MStar Semiconductor, Inc. All rights reserved.
// All software, firmware and related documentation herein ("MStar Software") are
// intellectual property of MStar Semiconductor, Inc. ("MStar") and protected by
// law, including, but not limited to, copyright law and international treaties.
// Any use, modification, reproduction, retransmission, or republication of all
// or part of MStar Software is expressly prohibited, unless prior written
// permission has been granted by MStar.
//
// By accessing, browsing and/or using MStar Software, you acknowledge that you
// have read, understood, and agree, to be bound by below terms ("Terms") and to
// comply with all applicable laws and regulations:
//
// 1. MStar shall retain any and all right, ownership and interest to MStar
//    Software and any modification/derivatives thereof.
//    No right, ownership, or interest to MStar Software and any
//    modification/derivatives thereof is transferred to you under Terms.
//
// 2. You understand that MStar Software might include, incorporate or be
//    supplied together with third party's software and the use of MStar
//    Software may require additional licenses from third parties.
//    Therefore, you hereby agree it is your sole responsibility to separately
//    obtain any and all third party right and license necessary for your use of
//    such third party's software.
//
// 3. MStar Software and any modification/derivatives thereof shall be deemed as
//    MStar's confidential information and you agree to keep MStar's
//    confidential information in strictest confidence and not disclose to any
//    third party.
//
// 4. MStar Software is provided on an "AS IS" basis without warranties of any
//    kind. Any warranties are hereby expressly disclaimed by MStar, including
//    without limitation, any warranties of merchantability, non-infringement of
//    intellectual property rights, fitness for a particular purpose, error free
//    and in conformity with any international standard.  You agree to waive any
//    claim against MStar for any loss, damage, cost or expense that you may
//    incur related to your use of MStar Software.
//    In no event shall MStar be liable for any direct, indirect, incidental or
//    consequential damages, including without limitation, lost of profit or
//    revenues, lost or damage of data, and unauthorized system use.
//    You agree that this Section 4 shall still apply without being affected
//    even if MStar Software has been modified by MStar in accordance with your
//    request or instruction for your use, except otherwise agreed by both
//    parties in writing.
//
// 5. If requested, MStar may from time to time provide technical supports or
//    services in relation with MStar Software to you for your use of
//    MStar Software in conjunction with your or your customer's product
//    ("Services").
//    You understand and agree that, except otherwise agreed by both parties in
//    writing, Services are provided on an "AS IS" basis and the warranty
//    disclaimer set forth in Section 4 above shall apply.
//
// 6. Nothing contained herein shall be construed as by implication, estoppels
//    or otherwise:
//    (a) conferring any license or right to use MStar name, trademark, service
//        mark, symbol or any other identification;
//    (b) obligating MStar or any of its affiliates to furnish any person,
//        including without limitation, you and your customers, any assistance
//        of any kind whatsoever, or any information; or
//    (c) conferring any license or right under any intellectual property right.
//
// 7. These terms shall be governed by and construed in accordance with the laws
//    of Taiwan, R.O.C., excluding its conflict of law rules.
//    Any and all dispute arising out hereof or related hereto shall be finally
//    settled by arbitration referred to the Chinese Arbitration Association,
//    Taipei in accordance with the ROC Arbitration Law and the Arbitration
//    Rules of the Association by three (3) arbitrators appointed in accordance
//    with the said Rules.
//    The place of arbitration shall be in Taipei, Taiwan and the language shall
//    be English.
//    The arbitration award shall be final and binding to both parties.
//
//******************************************************************************
//<MStar Software>

#define LOG_TAG "audio_hw_primary"
//#define LOG_NDEBUG 0

#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include <sys/types.h>             // for open
#include <sys/stat.h>
#include <fcntl.h>

#include <cutils/log.h>
#include <cutils/str_parms.h>
#include <cutils/properties.h>

#include <hardware/hardware.h>
#include <system/audio.h>
#include <hardware/audio.h>

#include <tinyalsa/asoundlib.h>
#include <audio_utils/resampler.h>

#include <apiAUDIO.h>
#include <drvAUDIO_if.h>


/* ALSA cards */
#define CARD_MAD        0
#define CARD_UAC_MIN    1
#define CARD_PRIMAX_MIC 2                          // primax bluetooth remote control microphone
#define CARD_UAC_MAX    4
#define CARD_SCO        5                          // Bluetooth Headset
#define CARD_DEFAULT    6
#define CARD_MAX       CARD_DEFAULT
#define DEVICE_SPEAKER    0
#define DEVICE_LINEOUT    1    //RCA line out
#define DEVICE_CAPTURE_DEVICE0 0
#define DEVICE_CAPTURE_DEVICE1 1

#define AUDIO_PARAMETER_LINEOUT_CONNECTED "lineout_connected"
#define AUDIO_PARAMETER_UAC_CONNECTED     "uac_connected"
#define AUDIO_PARAMETER_UAC_ENABLE        "uac_enable"
#define AUDIO_PARAMETER_UAC_RECORD        "uac_record"
#define AUDIO_PARAMETER_UAC_PROBE         "uac_probe"
#define AUDIO_PARAMETER_HEADSET_NAME      "bt_headset_name"
#define AUDIO_PARAMETER_PLAYBACK_STATUS   "playback_status"
#define AUDIO_PARAMETER_CAPTURE_STATUS    "capture_status"

#define UAC_IDENTIFY_LENGTH     14                 // 14B
#define NUM_RATE_CHANNEL        8                  // UAC output rate/channel combine

#define DEFAULT_OUT_SAMPLING_RATE 44100            // TODO: need compare with 48K
#define RESAMPLER_BUFFER_SIZE     96000            // 500ms
#define READ_FRAME                1024             // cap2uac read 4096B each loop
#define READCAPTURE_BUFFER_SIZE     192000     /*1s: 48000*2*2*/

enum {
    E_STOP = 0,
    E_PLAY,
    E_SUSPEND,
    E_RESUME,
};

struct mstar_audio_device {
    struct audio_hw_device device;

    pthread_mutex_t lock;                          /* see note below on mutex acquisition order */
    int uac_output;                                /* should support card index return by uac_probe */
    int uac_record;                                /* the uac card we use for recording */

    struct pcm_config config;
    struct pcm* pcm;                               // SPK capture for AEC/UAC
    char* far_buf;                                 // buffer hold request pcm data, after SRC
    char* cap_buf;                                 // buffer for capture pcm data, before SRC
    struct resampler_itfe* resampler;

    bool mic_mute;

};

struct mstar_stream_out {
    struct audio_stream_out stream;

    pthread_mutex_t lock; // see note below on mutex acquisition order
    struct mstar_audio_device* dev;
    struct pcm_config config;
    struct pcm* pcm;
};

struct mstar_stream_in {
    struct audio_stream_in stream;

    pthread_mutex_t lock; // see note below on mutex acquisition order
    int cards; // stream associate cards
    struct mstar_audio_device *dev;
    struct pcm_config config;
    struct pcm* pcm;

    struct resampler_itfe* resampler;
    int16_t* buffer; // buffer for SRC, PreProcessing
    unsigned int req_rate;
    unsigned int req_channels;
    unsigned int hw_rate; // ALSA Driver sample rate!
    unsigned int hw_channels; // for UAC Stereo MIC!

    char* buffer_rdcap1;  //buffer for read capture1
    char* prb1;    //pointer for in->buffer read capture1 buffer
    char* pwb1;   //pointer for pcm write capture1 buffer

    int state_indevice;
};

static int uac_card; // current UAC card index after scan /proc/asound
static int playback_status;
static int capture_status;
static int rate_channel[NUM_RATE_CHANNEL][2]={{48000,2},{32000,2},{16000,2},{8000,2},
                                              {48000,1},{32000,1},{16000,1},{8000,1}};

// if card=0, we search the first avail uac device
// else only check the specific card!
static int uac_connected(int flag, int card)
{
    int ret=0;
    int i, fd;
    int start, end;
    char fn[128];

    uac_card = 0;
    if (0 == card) {
        start = 1;
        end = CARD_UAC_MAX + 1;
    } else {
        start = card;
        end = card+1;
    }

    for (i = start; i < end; i++) {
        snprintf(fn, sizeof(fn), "/proc/asound/card%u/pcm0%c/info", i, flag&PCM_IN ? 'c':'p');
        fd = open(fn, O_RDONLY);
        if (fd > 0) {
            uac_card = i;
            close(fd);
            break;
        }
    }

    if (i == end) {
        ret = 0;
    } else {
        ret = 1;
    }

    return ret;
}

// return format: D:1:vvid:ppid;
// part1: D=capture+play, C=capture, P=playback
// part2: card number
// part3/4: USB vendor id & product id
static char* uac_probe(void)
{
    char *str, *p;
    int i, fdcap, fdplay, fdid;
    char fn[128];

    str = (char *)malloc(UAC_IDENTIFY_LENGTH*CARD_UAC_MAX);
    if (NULL == str) {
        return NULL;
    }

    memset(str, 0, UAC_IDENTIFY_LENGTH*CARD_UAC_MAX);
    p = str;
    for (i=1; i<CARD_UAC_MAX+1; i++) {
        snprintf(fn, sizeof(fn), "/proc/asound/card%u/pcm0c/info", i); // UAC start from card1
        fdcap = open(fn, O_RDONLY);
        snprintf(fn, sizeof(fn), "/proc/asound/card%u/pcm0p/info", i);
        fdplay = open(fn, O_RDONLY);
        if ((fdcap>0) && (fdplay>0)) {
            p[0] = 'D';
        } else if (fdcap > 0) {
            p[0] = 'C';
        } else if (fdplay > 0) {
            p[0] = 'P';
        } else {
            continue;
        }
        close(fdcap);
        close(fdplay);

        p[1] = ':';
        snprintf(p+2, 2, "%u", i);
        p[3] = ':';
        snprintf(fn, sizeof(fn), "/proc/asound/card%u/usbid", i);
        fdid = open(fn, O_RDONLY);
        if (fdid > 0) {
            read(fdid, p+4, 9);
            close(fdid);
        } else {
            memcpy(p+4, "ffff:ffff", 9);
        }
        p[13] = ';';
        p += UAC_IDENTIFY_LENGTH;
    }
    if (p == str) {
        ALOGV("uac_probe: NO UAC found.");
        free(str);
        return NULL;
    }

    p -= 1;
    p[0] = '\0';
    ALOGV("uac_probe: %s", str);
    return str;
}

static int alsa_open(uint32_t card, uint32_t dev, uint32_t flags, struct pcm **pcm, struct pcm_config *config)
{
    ALOGV("alsa open pcmC%uD%u%c ch %d sr %d.", card,dev,flags&PCM_IN?'c':'p',config->channels,config->rate);

    if ((NULL == pcm) || (NULL == config)) {
        return -1;
    }

    config->format = PCM_FORMAT_S16_LE;
    config->period_size = 1024;
    config->period_count = 4;
    config->start_threshold = 0;
    config->stop_threshold = 0;
    config->silence_threshold = 0;
    *pcm = pcm_open(card, dev, flags, config);
    if (!*pcm || !pcm_is_ready(*pcm)) {
        ALOGE("alsa open fail %s", pcm_get_error(*pcm));
        pcm_close(*pcm);
        *pcm = NULL;
        return -2;
    }

    return 0;
}

// the loop: read data from speaker, do SRC, send to usb audio class
//
void cap2uac_thread(void *dev)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)dev;
    int ret, i;
    struct pcm_config config_uac;
    struct pcm* pcm_uac = NULL;

    ALOGV("enter cap2uac thread");
    if (adev->uac_output && (1 == uac_connected(PCM_OUT, adev->uac_output))) {
        for (i = 0; i < NUM_RATE_CHANNEL; i++) {
            config_uac.rate = rate_channel[i][0];
            config_uac.channels = rate_channel[i][1];
            ret = alsa_open(uac_card, 0, PCM_OUT, &pcm_uac, &config_uac);
            if (0 == ret) {
                break;
            }
        }
        if (i >= NUM_RATE_CHANNEL) {
            ALOGE("cap2uac: open card %d fail!", uac_card);
            return;
        }
    }

    adev->config.channels = 2;
    adev->config.rate = 48000;
    ret = alsa_open(CARD_MAD, 1, PCM_IN, &adev->pcm, &adev->config);
    if (0 != ret) {
        ALOGE("cap2uac: open capture 2 fail!");
        pcm_close(pcm_uac);
        return;
    }
    if (config_uac.rate != adev->config.rate) {
        if (NULL != adev->resampler) {
            release_resampler(adev->resampler);
            adev->resampler = NULL;
        }
        ret = create_resampler(adev->config.rate, config_uac.rate, config_uac.channels, RESAMPLER_QUALITY_DEFAULT, NULL, &adev->resampler);
        if (0 != ret) {
            goto exit;
        }
    }

    while ((adev->uac_output) && (1 == uac_connected(PCM_OUT, adev->uac_output))) {
        ret = pcm_read(adev->pcm, adev->cap_buf, READ_FRAME*4);
        if (ret >= 0) {
            size_t in_frames = READ_FRAME;
            size_t out_frames = READ_FRAME;

            if (1 == config_uac.channels) { // UAC is mono, do stereo -> mono first
                int16_t* ptr = (int16_t *)adev->cap_buf;
                for (i = 0; i < READ_FRAME; i++) {
                    ptr[i] = (int16_t)(((int)ptr[i*2]+(int)ptr[i*2+1]) >> 1); // stereo -> mono
                }
            }
            if (NULL != adev->resampler) { // do SRC if uac samplerate != 48000
                adev->resampler->resample_from_input(adev->resampler, (int16_t *)adev->cap_buf, &in_frames,
                                                     (int16_t *)adev->far_buf, &out_frames);
            } else {
                memcpy(adev->far_buf, adev->cap_buf, out_frames<<config_uac.channels);
            }

            if (pcm_write(pcm_uac, (void *)adev->far_buf, out_frames<<config_uac.channels)) {
                ALOGE("cap2uac: uac pcm_write error!");
                usleep((out_frames*1000)/config_uac.rate); // write usb audio fail, sleep a while
            }
        } else {
            ALOGE("cap2uac: speaker pcm_read error!");
            usleep((READ_FRAME*1000)/48000); // read from speaker fail, sleep a while
        }
    }
    exit:
    pcm_close(pcm_uac);
    pcm_close(adev->pcm);
    adev->pcm = NULL;
    ALOGV("exit cap2uac thread");
}

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    return DEFAULT_OUT_SAMPLING_RATE;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    if (DEFAULT_OUT_SAMPLING_RATE == rate) {
        return 0;
    } else {
        return -EINVAL;
    }
}

// about 25ms for 44.1K, CDD5.4: output latency < 45ms
static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    return 1024*2*2;
}

static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    return AUDIO_CHANNEL_OUT_STEREO;
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, audio_format_t format)
{
    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    return 0;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    ALOGV("out_set_parameters: %s", kvpairs);

    return 0;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
#if ENABLE_DUAL_OUTPUT
    struct str_parms *parms = NULL;
    char *str = NULL;
    char value[32] = {0};
    int ret = 0;
    int card, fd, pcm;
    char fn[128];

    str = (char *)malloc(sizeof(char)*4);
    if (NULL == str) {
        return NULL;
    }
    memset(str, 0, sizeof(char)*4);

    ALOGV("adev_get_parameters: %s", keys);
    parms = str_parms_create_str(keys);
    if ( parms == NULL) {
        ALOGE("parms is null in %s",__FUNCTION__);
        return str;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_LINEOUT_CONNECTED, value, sizeof(value));
    if (ret >= 0 ) {
        ALOGV("%s ret %d",__FUNCTION__,ret);
        card = CARD_MAD;
        pcm = 1;  // pcm1 present RCA;pcm0 present speaker
        snprintf(fn, sizeof(fn), "/proc/asound/card%u/pcm%u%c/info", card,pcm,'p');
        fd = open(fn, O_RDONLY);
        if (fd > 0) {
            str[0] = '1';
            str[1] = 0;
            close(fd);
        }
        // MStar Android Patch Begin
        // 0 is stdin/stdout
        else if (fd == 0) {
            close(fd);
        }
        // MStar Android Patch End
    }
    str_parms_destroy(parms);

    return str;
#else
    return strdup("");
#endif
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
#ifdef BUILD_GOOGLETV
    struct mstar_stream_out *out = (struct mstar_stream_out *)stream;
    uint32_t latency = 0;

    latency = pcm_get_latency(out->pcm);

    if ( latency < 10 ) {
        latency = 10;
    }

    return latency;
#else
    ALOGV("out_get_latency ret 10ms");
    return 10;
#endif
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    return -ENOSYS;
}

static int check_mad_ready(size_t bytes)
{
    char property[PROPERTY_VALUE_MAX] = {0};

    if (property_get("mstar.audio.init", property, NULL) > 0) {
        if (atoi(property)) {
            return 0 ;
        }
    }

    ALOGE("check_mad_ready failed!");
    usleep(bytes * (1000000 / DEFAULT_OUT_SAMPLING_RATE / 4));

    return -1;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer, size_t bytes)
{
    struct mstar_stream_out *out = (struct mstar_stream_out *)stream;
    struct timeval tv1,tv2,tv;
    int i;
    static int iLoop = 0;
    char * cap_buf = (char *) buffer;

    iLoop++;
    if(iLoop%0x40)
        memset(buffer, 0, bytes);
    else{
        for(i = 0;i < bytes;i++)
            cap_buf[i] = (7*i)<<(i%6);
    }


    pthread_mutex_lock(&out->lock);


    if (E_SUSPEND == playback_status) {
        goto exit;
    } else if (E_PLAY != playback_status) {
        if (0 != check_mad_ready(bytes)) {
            goto exit;
        }

        playback_status = E_PLAY;
    }

    gettimeofday(&tv1,NULL);
    if (pcm_write(out->pcm, (void *)buffer, bytes)) {
        ALOGE("A2DP_DELAY7.out_write_mstar: MAD out_write error!");
    }
    gettimeofday(&tv2,NULL);
    timersub(&tv2,&tv1,&tv);
    if(cap_buf[0x40]|cap_buf[0x41]|cap_buf[0x42]|cap_buf[0x43])
    {
        ALOGE("A2DP_DELAY7.out_write_mstar2  (%d:%d) %d:%d",bytes,tv.tv_sec,tv.tv_usec,tv1.tv_sec,tv1.tv_usec);
        ALOGE("A2DP_DELAY7.out_write_mstar2 %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", \
            cap_buf[0x40],cap_buf[0x41],cap_buf[0x42],cap_buf[0x43],cap_buf[0x44],cap_buf[0x45],cap_buf[0x46],cap_buf[0x47], \
            cap_buf[0x48],cap_buf[0x49],cap_buf[0x4a],cap_buf[0x4b],cap_buf[0x4c],cap_buf[0x4d],cap_buf[0x4e],cap_buf[0x4f]);
    }
exit:
    pthread_mutex_unlock(&out->lock);

    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    return -EINVAL;
}

static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream,
                                        int64_t *timestamp)
{
    return -EINVAL;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct mstar_stream_in *in = (struct mstar_stream_in *)stream;

    ALOGV("in_get_sample_rate card %d ret %d", in->cards, in->req_rate);
    return in->req_rate; // We do SRC@HAL, so we return request rate
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return 0;
}

// CDD5.4: input latency < 50ms!
static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct mstar_stream_in *in = (struct mstar_stream_in *)stream;
    int frames;

    frames = in->req_rate / 100; // 10ms
    frames = ((frames + 15) / 16) * 16;

    ALOGV("in_get_buffer_size:10ms=%d frames", frames);
    return frames*audio_stream_frame_size((struct audio_stream *)stream);
}

static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    struct mstar_stream_in *in = (struct mstar_stream_in *)stream;
    int channels;

    channels = AUDIO_CHANNEL_IN_STEREO; // need handle BF case
    if (1 == in->req_channels) {
        channels = AUDIO_CHANNEL_IN_MONO;
    }

    return channels;
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int in_set_format(struct audio_stream *stream, audio_format_t format)
{
    return 0;
}

static int in_standby(struct audio_stream *stream)
{
    return 0;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    ALOGV("in_set_parameters: %s", kvpairs);

    return 0;
}

static char * in_get_parameters(const struct audio_stream *stream, const char *keys)
{
    ALOGV("in_get_parameters: %s", keys);

    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

//this function is thread's callback  ,the thread to read pcm
static void in_read_thread(void* stream)
{
    struct mstar_stream_in *in = (struct mstar_stream_in *)stream;
    int  bytes_rdcap1 = 0;
    int  ret ;

    ALOGV("start in_read_thread \n");
    bytes_rdcap1 = in_get_buffer_size((struct audio_stream *)in);
    if (NULL != in->resampler) {
        bytes_rdcap1 =  bytes_rdcap1 * in->hw_rate / in->req_rate;
        bytes_rdcap1 = ((bytes_rdcap1 + 15) / 16) * 16; // think again: 44.1K align???
        bytes_rdcap1 = bytes_rdcap1 * in->hw_channels / in->req_channels;
    }
    while (in->state_indevice) {
        if (E_PLAY == capture_status) {
            if ((in->pwb1 + bytes_rdcap1) > ( in->buffer_rdcap1 + READCAPTURE_BUFFER_SIZE)) {
                in->pwb1 = in->buffer_rdcap1;
            }
            ret = pcm_read(in->pcm, in->pwb1, bytes_rdcap1);
            if (0 == ret) {
                in->pwb1 = in->pwb1 + bytes_rdcap1;
            } else {
                usleep(5*1000);
            }
        } else {
            usleep(5*1000);
        }
    }

    in->state_indevice = 2;
    ALOGV("end in_read_thread \n");
}

// we handle 3 case:
// 1: bluetooth, must be 8K MONO, read data from socket
// 2: read data from input device, need handle SRC
// 3: preprocess, AEC read farend from 2rd capture, BF read stereo and output mono
static ssize_t in_read(struct audio_stream_in *stream, void* buffer, size_t bytes)
{
    int ret = 0;
    struct mstar_stream_in *in = (struct mstar_stream_in *)stream;
    struct mstar_audio_device *adev = in->dev;
    size_t frames = bytes/audio_stream_frame_size(&in->stream.common);
    int bytes_rd, bytes_near;
    int avail_bytes;

    pthread_mutex_lock(&in->lock);

    if (E_SUSPEND == capture_status) {
        goto exit;
    } else if (E_PLAY != capture_status) {
        if (0 != check_mad_ready(bytes)) {
            goto exit;
        }

        capture_status = E_PLAY;
    }

    bytes_near = bytes;
    if (NULL != in->resampler) {
        bytes_rd = bytes * in->hw_rate / in->req_rate;
        bytes_rd = ((bytes_rd + 15) / 16) * 16; // think again: 44.1K align???
        bytes_rd = (bytes_rd * in->hw_channels) / in->req_channels; // BF: hw_channels=2, req_channels=1
    } else {
        bytes_rd  = bytes;
    }

    avail_bytes = in->pwb1 - in->prb1;
    if (avail_bytes < 0) {
        avail_bytes = avail_bytes + READCAPTURE_BUFFER_SIZE;
    }
    if (avail_bytes >= (READCAPTURE_BUFFER_SIZE - bytes_rd)) {
        ALOGE("the capture1 buffer overflow!pwb1 = %p ,prb1 = %p in->buffer_rdcap1 = %p\n",in->pwb1,in->prb1,in->buffer_rdcap1);
    } else if (avail_bytes < bytes_rd) {
        int sleeptime1 = 0;

        while (avail_bytes < bytes_rd ) {
            if (sleeptime1 >= 50) {
                ALOGE("in read: pcm_read capture error!, zero buffer\n");
                memset(buffer, 0, bytes);
                goto exit;
            }
            usleep(2*1000);
            sleeptime1++;
            avail_bytes = in->pwb1 - in->prb1;
            avail_bytes = (avail_bytes < 0)?(avail_bytes+READCAPTURE_BUFFER_SIZE):avail_bytes;
        }
    }
    if ((in->prb1 + bytes_rd ) > (in->buffer_rdcap1 + READCAPTURE_BUFFER_SIZE)) {
        in->prb1 = in->buffer_rdcap1;
    }
    memcpy(in->buffer, in->prb1, bytes_rd);
    in->prb1 = in->prb1 + bytes_rd;

    if (NULL != in->resampler) {
        size_t in_frames = frames*in->hw_rate/in->req_rate;
        size_t out_frames = frames;
        in->resampler->resample_from_input(in->resampler, in->buffer, &in_frames,
                                           in->buffer+bytes_rd/2, &out_frames);
        bytes_near = out_frames*2*in->hw_channels;
        memcpy(in->buffer, in->buffer+bytes_rd/2, bytes_near); // request sample pcm data @ in->buffer
    }

    if (adev->mic_mute) {
        memset(buffer, 0, bytes);
    } else {
        memcpy(buffer, in->buffer, bytes);
    }

    exit:
    pthread_mutex_unlock(&in->lock);

    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out)
{
    struct mstar_audio_device *ladev = (struct mstar_audio_device *)dev;
    struct mstar_stream_out *out;

    ALOGV("adev_open_output_stream dev:%d fmt/chn/sr %d %d %d", devices, config->format,config->channel_mask, config->sample_rate);
    out = (struct mstar_stream_out *)calloc(1, sizeof(struct mstar_stream_out));
    if (NULL == out) {
        return -ENOMEM;
    }
    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;
    out->dev = ladev;

    config->format = out_get_format(&out->stream.common); // return UAC format!
    config->channel_mask = out_get_channels(&out->stream.common);
    config->sample_rate = out_get_sample_rate(&out->stream.common);

    out->config.channels = 2;
    out->config.rate = config->sample_rate;
    playback_status = E_STOP;

#if ENABLE_DUAL_OUTPUT
    if(devices & AUDIO_DEVICE_OUT_LINE_OUT)
    {
        alsa_open(CARD_MAD, DEVICE_LINEOUT, PCM_OUT, &out->pcm, &out->config);
    }
    else
    {
        alsa_open(CARD_MAD, DEVICE_SPEAKER, PCM_OUT, &out->pcm, &out->config);
    }
    *stream_out = &out->stream;

    return 0;
#else
    alsa_open(CARD_MAD, DEVICE_SPEAKER, PCM_OUT, &out->pcm, &out->config);
    *stream_out = &out->stream;

    return 0;
#endif
}

static void adev_close_output_stream(struct audio_hw_device *dev, struct audio_stream_out *stream)
{
    struct mstar_stream_out *out = (struct mstar_stream_out *)stream;

    playback_status = E_STOP;

    if (NULL != out) {
        if (out->pcm) {
            pcm_close(out->pcm);
        }
        free(stream);
    }
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)dev;
    struct str_parms *parms = NULL;
    char value[32] = {0};
    int ret = 0;

    ALOGV("adev_set_parameters: %s", kvpairs);
    if (NULL == adev)
        return 0;

    parms = str_parms_create_str(kvpairs);
    if (NULL == parms) {
        ALOGE("parms is null in %s",__FUNCTION__);
        return -ENOMEM;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_UAC_ENABLE, value, sizeof(value));
    if (ret >= 0) {
        int uac = 0;

        if ((strcmp(value, "on")==0) && uac_connected(PCM_OUT, 0)) {
            uac = uac_card;
        } else if ((value[0] > '0') && (value[0] < '5')) {
            uac = value[0] - '0';
            if (0 == uac_connected(PCM_OUT, uac)) { // ignore is uac no connect
                uac = 0;
                adev->uac_output = 0;
            }
        } else {
            uac = 0;
            adev->uac_output = 0;
        }

        if ((uac > 0) && (adev->uac_output == 0)) { // start thread iff no thread run! ignore any new card setting
            pthread_t tthread;
            adev->uac_output = uac;
            pthread_create(&tthread, NULL, (void *)cap2uac_thread, adev);
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_UAC_RECORD, value, sizeof(value));
    if (ret >= 0) {
        int card;

        card = atol(value);
        if ((card<0) || (card>CARD_MAX)) {
            card = CARD_DEFAULT;
        }
        adev->uac_record = card;
    }

    if ( strcmp(kvpairs, "DualAudioOn") == 0 ) {
        OUTPUT_SOURCE_INFO SourceInfo;

        ALOGE("%s: Turn on sub audio path to 2nd DMA Reader.\r\n",__FUNCTION__);

        memset((void *) &SourceInfo, 0, sizeof(OUTPUT_SOURCE_INFO));
        MApi_AUDIO_GetOutputSourceInfo(&SourceInfo);

        SourceInfo.HpOut = E_CONNECT_SUB;

        MApi_AUDIO_SetOutputSourceInfo(&SourceInfo);
        MApi_AUDIO_InputSwitch(AUDIO_DSP4_DVB_INPUT, E_AUDIO_GROUP_SUB);
    }
    else if ( strcmp(kvpairs, "DualAudioOff") == 0 ) {
        OUTPUT_SOURCE_INFO SourceInfo;

        ALOGE("%s: Turn off sub audio path to 2nd DMA Reader.\r\n",__FUNCTION__);

        memset((void *) &SourceInfo, 0, sizeof(OUTPUT_SOURCE_INFO));
        MApi_AUDIO_GetOutputSourceInfo(&SourceInfo);

        SourceInfo.HpOut = SourceInfo.SpeakerOut;

        MApi_AUDIO_SetOutputSourceInfo(&SourceInfo);
        MApi_AUDIO_InputSwitch(AUDIO_NULL_INPUT, E_AUDIO_GROUP_SUB);
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_PLAYBACK_STATUS, value, sizeof(value));//for STR
    if (ret >= 0) {
        if ((strcmp(value, "suspend")==0)) {
            playback_status = E_SUSPEND;
        }
        else if ((strcmp(value, "resume")==0)) {
            playback_status = E_RESUME;
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_CAPTURE_STATUS, value, sizeof(value));//for STR
    if (ret >= 0) {
        if ((strcmp(value, "suspend")==0)) {
            capture_status = E_SUSPEND;
        }
        else if ((strcmp(value, "resume")==0)) {
            capture_status = E_RESUME;
        }
    }

    str_parms_destroy(parms);

    return ret;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)dev;
    struct str_parms *parms = NULL;
    char *str = NULL;
    char value[32] = {0};
    int ret = 0;

    str = (char *)malloc(sizeof(char)*4);
    if (NULL == str) {
        return NULL;
    }
    memset(str, 0, 4);

    ALOGV("adev_get_parameters: %s", keys);
    parms = str_parms_create_str(keys);
    if ( parms == NULL) {
        ALOGE("parms is null in %s",__FUNCTION__);
        return str;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_UAC_CONNECTED, value, sizeof(value));
    if (ret >= 0) {
        int out = uac_connected(PCM_OUT,0);
        int in  = uac_connected(PCM_IN,0);

        if (in && out) {
            snprintf(str, sizeof(str), "3");
        } else if (out) {
            snprintf(str, sizeof(str), "1");
        } else if (in) {
            snprintf(str, sizeof(str), "2");
        } else {
            snprintf(str, sizeof(str), "0");
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_UAC_RECORD, value, sizeof(value));
    if (ret >= 0) {
        if ((adev->uac_record<0) || (adev->uac_record>CARD_MAX)) {
            str[0] = '0' + CARD_DEFAULT;
        } else {
            str[0] = '0' + adev->uac_record;
        }
        str[1] = 0;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_UAC_PROBE, value, sizeof(value));
    if (ret >= 0) {
        char* list = uac_probe();
        if (list != NULL) {
            str_parms_destroy(parms);
            free(str);
            return list;
        }
    }

    str_parms_destroy(parms);

    return str;
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    return 0;
}

static int adev_get_master_volume(struct audio_hw_device *dev,
                                  float *volume)
{
    return -ENOSYS;
}

static int adev_set_master_mute(struct audio_hw_device *dev, bool muted)
{
    return 0;
}

static int adev_get_master_mute(struct audio_hw_device *dev, bool *muted)
{
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)dev;

    adev->mic_mute = state;

    return 0;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)dev;

    *state = adev->mic_mute;

    return 0;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
                                         const struct audio_config *config)
{
    int frames;
    int channel_count = popcount(config->channel_mask);

    ALOGV("adev_get_input_buffer_size fmt/chn/sr %d %d %d", config->format, channel_count, config->sample_rate);

    frames = config->sample_rate/100; // 10ms
    frames = ((frames+15)/16) * 16;

    return frames*channel_count*2;
}

static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle,
                                  audio_devices_t devices,
                                  struct audio_config *config,
                                  struct audio_stream_in **stream_in)
{
    struct mstar_audio_device *ladev = (struct mstar_audio_device *)dev;
    struct mstar_stream_in *in;
    int ret;
    int channel_count = popcount(config->channel_mask);
    int device = DEVICE_CAPTURE_DEVICE0;

    devices &= ~AUDIO_DEVICE_BIT_IN;
    ALOGV("adev_open_input_stream dev:%x fmt/chn/sr %d %x %d", devices, config->format, config->channel_mask, config->sample_rate);
    *stream_in = NULL;
    if ((config->format!=AUDIO_FORMAT_PCM_16_BIT) || (channel_count<1) || (channel_count>2)) {
        return -EINVAL;
    }
    if ((config->sample_rate<8000) || (config->sample_rate>48000)) {
        return -EINVAL;
    }

    in = (struct mstar_stream_in *)calloc(1, sizeof(struct mstar_stream_in));
    if (NULL == in) {
        return -ENOMEM;
    }
    in->buffer = (int16_t *)malloc(RESAMPLER_BUFFER_SIZE); // buffer for SRC, PreProcessing
    if (NULL == in->buffer) {
        free(in);
        return -ENOMEM;
    }
    in->buffer_rdcap1 = (char *)malloc(READCAPTURE_BUFFER_SIZE);
    if (NULL == in->buffer_rdcap1) {
        free(in->buffer);
        free(in);
        return -ENOMEM;
    }

    in->prb1 = in->pwb1 = in->buffer_rdcap1;
    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;
    in->dev = ladev;


    if (CARD_DEFAULT == ladev->uac_record) {
        if (0 == uac_connected(PCM_IN, 0)) {
            in->cards = CARD_MAD;
        } else {
            in->cards = uac_card;
        }
    } else {
        in->cards = ladev->uac_record;
        ladev->uac_record = CARD_DEFAULT;
    }

    in->req_rate = config->sample_rate; // div0: between 8000-48000
    in->req_channels = channel_count; // div0: 1 or 2

    if (CARD_SCO == in->cards) {
        ALOGI("adev_open_input_stream CARD_SCO\n");
    } else {
        if (CARD_MAD == in->cards) {
            if (1 == channel_count) {
                config->channel_mask = AUDIO_CHANNEL_IN_LEFT | AUDIO_CHANNEL_IN_RIGHT;
                ret = -EINVAL;
                goto exit;
            }
            in->config.channels = 2;
            in->config.rate = 48000;
            if (devices & (AUDIO_DEVICE_IN_CAPTURE_DEVICE0 & ~AUDIO_DEVICE_BIT_IN)) {
                device = DEVICE_CAPTURE_DEVICE0;
            } else if (devices & (AUDIO_DEVICE_IN_CAPTURE_DEVICE1 & ~AUDIO_DEVICE_BIT_IN)) {
                device = DEVICE_CAPTURE_DEVICE1;
            }
            if (0 != alsa_open(CARD_MAD, device, PCM_IN, &in->pcm, &in->config)) {
                ret = -ENODEV;
                goto exit;
            }
            in->hw_rate = in->config.rate;
            in->hw_channels = 2;
        } else {
            in->config.channels = in->req_channels;;
            in->config.rate = in->req_rate;
            // if card is primax bluetooth mic, the configuration set to a fixed value
            if (CARD_PRIMAX_MIC == in->cards) {
                in->config.rate = 16000;
                in->config.channels = 1;
            }
            if (0 != alsa_open(in->cards, 0, PCM_IN, &in->pcm, &in->config)) { // capture auto select first samplerate!
                if (1 == in->config.channels) {
                    in->config.channels = 2;
                } else {
                    in->config.channels = 1;
                }
                if (0 != alsa_open(in->cards, 0, PCM_IN, &in->pcm, &in->config)) { // Try STEREO for Konka UAC/UVC
                    ret = -ENODEV;
                    goto exit;
                }
            }

            in->hw_rate = in->config.rate;
            in->hw_channels = in->config.channels;
            if (in->hw_channels != in->req_channels) { // we don't handle STEREO<->MONO, let framework do it!
                if (in->hw_channels == 2) {
                    config->channel_mask = AUDIO_CHANNEL_IN_LEFT | AUDIO_CHANNEL_IN_RIGHT;
                } else {
                    config->channel_mask = AUDIO_CHANNEL_IN_FRONT;
                }
                pcm_close(in->pcm);
                ret = -EINVAL;
                goto exit;
            }
        }

        if (in->req_rate != in->hw_rate) {
            ret = create_resampler(in->hw_rate, in->req_rate, in->hw_channels, RESAMPLER_QUALITY_DEFAULT, NULL, &in->resampler);
            ALOGV("adev_open_input_stream SRC %d->%d,ch:%d", in->hw_rate, in->req_rate, in->hw_channels);
            if (ret != 0) {
                pcm_close(in->pcm);
                ret = -ENODEV;
                goto exit;
            }
        }
    }

    capture_status = E_STOP;
    in->state_indevice = 1;
    pthread_t tthread;
    ret = pthread_create(&tthread, NULL, (void *)in_read_thread, (void*)in);
    if (0 != ret ) {
        goto exit;
    }

    *stream_in = &in->stream;
    return 0;

    exit:
    free(in->buffer);
    free(in->buffer_rdcap1);
    free(in);
    return ret;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
                                    struct audio_stream_in *in)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)dev;
    struct mstar_stream_in *stream = (struct mstar_stream_in *)in;
    int exit_counter = 0;

    ALOGV("adev_close_input_stream");

    if ((NULL == adev) || (NULL == stream)) {
        return;
    }

    capture_status = E_STOP;

    stream->state_indevice = 0;
    do {
        if (stream->state_indevice == 2)
            break;

        exit_counter++;
        if (exit_counter > 500 * 60) {
            ALOGV("adev_close_input_stream : exit in_read_thread() timeout!");
            break;
        }

        usleep(2000);
    } while(1);  // wait in_read_thread exit, or buffer/pcm freed!

    if (stream->buffer) {
        free(stream->buffer);
        stream->buffer = NULL;
    }
    if (stream->buffer_rdcap1) {
        free(stream->buffer_rdcap1);
        stream->buffer_rdcap1 = NULL;
    }
    if (stream->resampler) {
        release_resampler(stream->resampler);
    }
    if (stream->pcm) {
        pcm_close(stream->pcm);
    }
    if ((0 == adev->uac_output) && adev->pcm) { // iff cap2uac no run!
        pcm_close(adev->pcm);
        adev->pcm = NULL;
    }

    free(stream);
    stream = NULL;

    return;
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    return 0;
}

static int adev_close(hw_device_t *device)
{
    struct mstar_audio_device *adev = (struct mstar_audio_device *)device;

    if (adev->far_buf) {
        free(adev->far_buf);
    }
    free(device);
    return 0;
}


static int adev_open(const hw_module_t* module, const char* name, hw_device_t** device)
{
    struct mstar_audio_device* adev;

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0) {
        return -EINVAL;
    }

    ALOGV("adev_open -------------------- build %s %s\n", __DATE__, __TIME__);
    adev = (struct mstar_audio_device*)calloc(1, sizeof(struct mstar_audio_device));
    if (NULL == adev) {
        return -ENOMEM;
    }

    adev->device.common.tag = HARDWARE_DEVICE_TAG;
    adev->device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->device.common.module = (struct hw_module_t *) module;
    adev->device.common.close = adev_close;

    adev->device.init_check = adev_init_check;
    adev->device.set_voice_volume = adev_set_voice_volume;
    adev->device.set_master_volume = adev_set_master_volume;
    adev->device.get_master_volume = adev_get_master_volume;
    adev->device.set_master_mute = adev_set_master_mute;
    adev->device.get_master_mute = adev_get_master_mute;
    adev->device.set_mode = adev_set_mode;
    adev->device.set_mic_mute = adev_set_mic_mute;
    adev->device.get_mic_mute = adev_get_mic_mute;
    adev->device.set_parameters = adev_set_parameters;
    adev->device.get_parameters = adev_get_parameters;
    adev->device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->device.open_output_stream = adev_open_output_stream;
    adev->device.close_output_stream = adev_close_output_stream;
    adev->device.open_input_stream = adev_open_input_stream;
    adev->device.close_input_stream = adev_close_input_stream;
    adev->device.dump = adev_dump;

    adev->uac_output = 0;
    adev->uac_record = CARD_DEFAULT;
    adev->resampler = NULL;

    adev->far_buf = (char *)malloc(RESAMPLER_BUFFER_SIZE);
    if (NULL == adev->far_buf) {
        adev->cap_buf = NULL;
        ALOGE("far buf malloc fail, cap2uac/aec no support!");
    } else {
        memset(adev->far_buf, 0, RESAMPLER_BUFFER_SIZE);
        adev->cap_buf = adev->far_buf + RESAMPLER_BUFFER_SIZE/2;
    }

    *device = &adev->device.common;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "MStar audio HW HAL",
        .author = "The Android Open Source Project",
        .methods = &hal_module_methods,
    },
};
