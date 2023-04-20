#include <iostream>
#include <fstream>
#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <string.h>
#include <malloc.h>
#include <lame/lame.h>
#include <alsa/asoundlib.h>
#include <chrono>

#include "audition/audition.hpp"


using std::cout;
using std::endl;


#define DEFAULT_FORMAT		\
{\
	WAVE_FORMAT_PCM,	\
	1,			\
	16000,			\
	32000,			\
	2,			\
	16,			\
	sizeof(WAVEFORMATEX)	\
}


typedef struct
{
    char rld[4];            
    int rLen;
    char wld[4];            
    char fld[4];           
    int fLen;              

    short wFormatTag;     
    short wChannels;       
    int nSamplesPersec ;    
    int nAvgBitsPerSample;  
    short wBlockAlign;     
    short wBitsPerSample;   

    char dld[4];           
    int wSampleLength;     

}WavHeader;


const char * RAW_PCM_SAVE_PATH              = "./resource/audio/awake_raw.pcm";
const char * CUTTED_PCM_SAVE_PATH           = "./resource/audio/awake_cut.pcm";
const char * CUTTED_MP3_SAVE_PATH           = "./resource/audio/awake.mp3";

const int FILE_OFFSET                       = 16000 * 2;
const int SAMPLING_RATE                     = 16000;
const int FRAME_PER_DENOISE_PROCESS         = 256;

const char * AWAKE_REPLY_AUDIO_PATH         = "./resource/audio/awake_reply.wav";
const char * ASR_REPLY_AUDIO_PATH           = "./resource/audio/asr_reply.wav";

const char * DENOISE_RAW_AUDIO_PATH         = "./resource/audio/denoise_raw.pcm";
const char * DENOISE_SAVE_AUDIO_PATH        = "./resource/audio/denoise.pcm";
const char * DENOISE_BACKGROUND_NOISE_PATH  = "./resource/audio/background_noise.pcm";
const char * DENOISE_PROCESS_AUDIO_PATH     = "./resource/audio/denoise_process.pcm";

static struct recorder * awake_iat;
static int awake_record_state = CONTINUE;
static int _is_awake = false;
static FILE * awake_audio_file = nullptr;

static FILE * denoise_raw_file = nullptr;
static FILE * denoise_save_file = nullptr;
static FILE * denoise_background_noise_file = nullptr;
static FILE * denoise_process_file = nullptr;

static char asr_params[MAX_PARAMS_LEN] = "";
static char * asr_result = nullptr;
static unsigned int asr_buffersize = BUFFER_SIZE;
static struct speech_rec asr_iat;

// auto start_time = std::chrono::system_clock::now();

// function for denoise
static void denoise_save_audio_file(char * audio_data, unsigned int len) {
    denoise_save_file = fopen(DENOISE_SAVE_AUDIO_PATH, "ab");
    if (nullptr == denoise_save_file)
        cout << "Denoise Save File Open Failed" << endl;
    fwrite(audio_data, sizeof(char), len, denoise_save_file);
    fclose(denoise_save_file);
}

static void denoise_for_audio_clip(char * audio_data, unsigned long len) {
    unsigned long rc = 0;
    unsigned long background_len = 0;
    
    denoise_background_noise_file = fopen(DENOISE_BACKGROUND_NOISE_PATH, "rb");
    if (nullptr == denoise_background_noise_file)
        cout << "Denoise Background File Open Failed 1" << endl;
    denoise_raw_file = fopen(DENOISE_RAW_AUDIO_PATH, "wb");
    if (nullptr == denoise_raw_file)
        cout << "Denoise Raw File Open Failed 1" << endl;
    
    fseek(denoise_background_noise_file, 0L, SEEK_END);
    background_len = ftell(denoise_background_noise_file);
    fseek(denoise_background_noise_file, 0L, SEEK_SET);

    char * buffer = (char*)malloc(background_len+len+1);
    if (nullptr == buffer)
        cout << "Denoise Malloc Error" << endl;
    rc = fread(buffer, sizeof(char), background_len, denoise_background_noise_file);
    if (rc != background_len)
        cout << "Denoise Buffer Read Error" << endl;
    for (unsigned int i = 0; i < len; ++i)
        buffer[i+background_len] = audio_data[i];

    rc = fwrite(buffer, sizeof(char), background_len+len, denoise_raw_file);
    if (rc != background_len+len)
        cout << "Denoise Raw File Short Write Error" << endl;
    
    fclose(denoise_background_noise_file);
    fclose(denoise_raw_file);
    // free(buffer);

    denoise_raw_file = fopen(DENOISE_RAW_AUDIO_PATH, "rb");
    if (nullptr == denoise_raw_file)
        cout << "Denoise Raw File Open Failed 2" << endl;
    // clear the denoise_process_file
    denoise_process_file = fopen(DENOISE_PROCESS_AUDIO_PATH, "wb");
    if (nullptr == denoise_raw_file)
        cout << "Denoise Process File Open Failed 1" << endl;
    fclose(denoise_process_file);
    denoise_process_file = fopen(DENOISE_PROCESS_AUDIO_PATH, "ab");
    if (nullptr == denoise_raw_file)
        cout << "Denoise Process File Open Failed 2" << endl;

    short int speex_buffer[FRAME_PER_DENOISE_PROCESS];
    SpeexPreprocessState * state = speex_preprocess_state_init(FRAME_PER_DENOISE_PROCESS, SAMPLING_RATE);
    int denoise = 1;
    int noise_suppress = -25;
    speex_preprocess_ctl(state, SPEEX_PREPROCESS_SET_DENOISE, &denoise);
    speex_preprocess_ctl(state, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &noise_suppress);


    while (1) {
        size_t read_size = fread(speex_buffer, sizeof(short int), FRAME_PER_DENOISE_PROCESS, denoise_raw_file);
        if (0 == read_size)
            break;
        speex_preprocess_run(state, (spx_int16_t*)(speex_buffer));
        rc = fwrite(speex_buffer, sizeof(spx_int16_t), read_size, denoise_process_file);
        if (rc != read_size)
            cout << "Denoise Process File Short Write Error" << endl;
    }
    fclose(denoise_raw_file);
    fclose(denoise_process_file);

    denoise_process_file = fopen(DENOISE_PROCESS_AUDIO_PATH, "rb");
    if (nullptr == denoise_process_file)
        cout << "Denoise Process File Open Failed 2" << endl;
    
    rc = fread(buffer, sizeof(char), background_len+len, denoise_process_file);
    if (rc != background_len+len)
        cout << "Denoise Process File Read Error" << endl;
    for (unsigned int i = 0; i < len; ++i)
        audio_data[i] = buffer[background_len+i];
    
    fclose(denoise_process_file);
    free(buffer);
}

static void denoise_clean_audio_file() {
    denoise_save_file = fopen(DENOISE_SAVE_AUDIO_PATH, "wb");
    if (nullptr == denoise_save_file)
        cout << "Denoise Clean File Failed" << endl;
    fclose(denoise_save_file);
}


// function for awake
void keywordDetection() {
    char sse_hints[128] = {0};
    WAVEFORMATEX fmt = DEFAULT_FORMAT;

    // denoise_clean_audio_file();

    _error = create_recorder(&awake_iat);
    if (SUCCESS != _error || nullptr == awake_iat) {
        cout << "Alsa Create Recorder Failed!" << _error << endl;
        goto exit;
    }

    _error = open_recorder(awake_iat, get_default_input_dev(), &fmt);
    if (SUCCESS != _error) {
        cout << "Alsa Open Recorder Failed!" << _error << endl;
        goto exit;
    }

    _error = start_record(awake_iat);
    if (SUCCESS != _error) {
        cout << "Alsa Start Record Failed!" << _error << endl;
        goto exit;
    }

    awake_record_state = FIRST;

    while (LAST != awake_record_state) {
        sleep(0.1);
    }

    snprintf(sse_hints, sizeof(sse_hints), "success");

exit:
    if (nullptr != awake_iat) {
        if (!is_record_stopped(awake_iat))
            stop_record(awake_iat);
        close_recorder(awake_iat);
        destroy_recorder(awake_iat);
        awake_iat = nullptr;
    }
}


int awakeIsWakeUp() {
    return _is_awake;
}


void awakeSetWakeup(int state) {
    _is_awake = state;
}


// function for ASR
void asr_on_result(const char * result, char is_last) {
    if (result) {
        size_t left = asr_buffersize - 1 - strlen(asr_result);
        size_t size = strlen(result);
        if (left < size) {
            asr_result = (char *)realloc(asr_result, asr_buffersize + BUFFER_SIZE);
            if (asr_result)
                asr_buffersize += BUFFER_SIZE;
            else {
                cout << "memory location error" << endl;
                return;
            }
        }
        strncat(asr_result, result, size);
        asr_show_result(asr_result, is_last);
    }
}


void asr_on_speech_begin() {
    if (asr_result) 
        free(asr_result);
    
    asr_result = (char *)malloc(BUFFER_SIZE);
    asr_buffersize = BUFFER_SIZE;
    memset(asr_result, 0, asr_buffersize);

    cout << "start listening" << endl;
}


void asr_on_speech_end(int reason) {
    if (reason == END_REASON_VAD_DETECT)
        cout << "Speaking dnoe!" << endl;
    else
        cout << "Recognizer Error!" << reason << endl;
}


static struct speech_rec_notifier asr_recnotifier = {
    asr_on_result,
    asr_on_speech_begin,
    asr_on_speech_end
};


void asrStartRecord() {
    int _error = -1;
    _error = sr_start_listening(&asr_iat);
    if (SUCCESS != _error) {
        cout << "Start Listening Failed!" << _error << endl;
        return;
    }
}


void asrFinishRecord() {
    int _error = -1;
    _error = sr_stop_listening(&asr_iat);
    if (SUCCESS != _error) {
        cout << "ASR Stop Listening Failed!" << _error << endl;
        return;
    }
}


int asrIsFinish() {
    return sr_is_asr_finish();
}


// function for audio processing 
void pcmCutRawAudio() {
    size_t readSize, rawFileSize;

    char pcm_buffer[FILE_OFFSET];

    FILE * rawFile = fopen(RAW_PCM_SAVE_PATH, "rb");
    if (NULL == rawFile) {
        cout << "PCM Raw File Open Failed!" << endl;
        return;
    }

    FILE * pcmFile = fopen(CUTTED_PCM_SAVE_PATH, "wb");
    if (NULL == pcmFile) {
        cout << "PCM Cutted File Open Failed!" << endl;
        return;
    }

    fseek(rawFile, 0, SEEK_END);
    rawFileSize = ftell(rawFile);

    if (rawFileSize <= FILE_OFFSET) {
        readSize = rawFileSize;
        fseek(rawFile, 0, SEEK_SET);
    }
    else {
        readSize = FILE_OFFSET;
        fseek(rawFile, -FILE_OFFSET, SEEK_END);
    }

    fread(pcm_buffer, sizeof(char), readSize, rawFile);
    fwrite(pcm_buffer, sizeof(char), readSize, pcmFile);

    fclose(pcmFile);
    fclose(rawFile);
}


void pcmConvPCM2MP3() {
    size_t readSize, writeSize;

    FILE * pcmFile = fopen(CUTTED_PCM_SAVE_PATH, "rb");
    if (NULL == pcmFile) {
        cout << "CONV Pcm File Open Failed!" << endl;
        return;
    }

    FILE * mp3File = fopen(CUTTED_MP3_SAVE_PATH, "wb");
    if (NULL == mp3File) {
        cout << "CONV Mp3 File Open Failed!" << endl;
        return;
    }

    const int PCM_BUFF_SIZE = 8192;
    const int MP3_BUFF_SIZE = 8192;

    short int pcm_buffer[PCM_BUFF_SIZE];
    unsigned char mp3_buffer[MP3_BUFF_SIZE];

    lame_t lame = lame_init();
    lame_set_in_samplerate(lame, 16000); 
    lame_set_quality(lame, 5);

    lame_set_VBR(lame, vbr_default);

    lame_set_num_channels(lame, 1);
    lame_init_params(lame);

    do {
        readSize = fread(pcm_buffer, sizeof(short int), PCM_BUFF_SIZE, pcmFile);

        if (readSize == 0)
            writeSize = lame_encode_flush(lame, mp3_buffer, MP3_BUFF_SIZE);
        else 
            writeSize = lame_encode_buffer(lame, pcm_buffer, pcm_buffer, (int)readSize, mp3_buffer, MP3_BUFF_SIZE);
    
        fwrite(mp3_buffer, writeSize, 1, mp3File);
    } while (readSize != 0);

    lame_close(lame);
    fclose(mp3File);
    fclose(pcmFile);
}


// function for play audio 
void play_set_pcm(FILE * fp, WavHeader * wav_header) {
    int rc;
    int ret;
    int size;
    snd_pcm_t* handle; 
    snd_pcm_hw_params_t* params;
    unsigned int val;
    int dir=0;
    snd_pcm_uframes_t frames;
    char *buffer;
    int channels=wav_header->wChannels;
    int frequency=wav_header->nSamplesPersec;
    int bit=wav_header->wBitsPerSample;
    int datablock=wav_header->wBlockAlign;

    rc=snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    if(rc<0)
    {
        perror("\nopen PCM device failed:");
        exit(1);
    }


    snd_pcm_hw_params_alloca(&params); 
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_alloca:");
        exit(1);
    }
     rc=snd_pcm_hw_params_any(handle, params);
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_any:");
        exit(1);
    }
    rc=snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED); 
    if(rc<0)
    {
        perror("\nsed_pcm_hw_set_access:");
        exit(1);
    }

    switch(bit/8)
    {
        case 1:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
                break ;
        case 2:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
                break ;
        case 3:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
                break ;

    }
    rc = snd_pcm_hw_params_set_channels(handle, params, channels);
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_set_channels:");
        exit(1);
    }
    val = frequency;
    rc = snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_set_rate_near:");
        exit(1);
    }

    rc = snd_pcm_hw_params(handle, params);
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params: ");
        exit(1);
    }

    rc = snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_get_period_size:");
        exit(1);
    }

    size = frames * datablock;

    buffer =(char*)malloc(size);
    fseek(fp,58,SEEK_SET); 

    while (1)
    {
        memset(buffer, 0, size);
		// memset(buffer, 0, sizeof(buffer));
        ret = fread(buffer, 1, size, fp);
        if(ret == 0)
            break;
         else if (ret != size)
            break;

        snd_pcm_writei(handle, buffer, frames);
        if (ret == -EPIPE){
          /* EPIPE means underrun */
          fprintf(stderr, "underrun occurred\n");
          snd_pcm_prepare(handle);
        }else if (ret < 0){
           fprintf(stderr,
           "error from writei: %s\n",
           snd_strerror(ret));
        }/*else if(ret > 0 || r == -EAGAIN){
          snd_pcm_wait
        }*/
        usleep(2 * 1000);
    }

    snd_pcm_drain(handle);
    snd_pcm_close(handle);
    free(buffer);
}


void playReplyAudio(int audioCase) {
    FILE * fp;
    WavHeader wav_header;

    switch (audioCase) {
        case 0:
            fp = fopen(AWAKE_REPLY_AUDIO_PATH, "rb");
            break;
        
        case 1:
            fp = fopen(ASR_REPLY_AUDIO_PATH, "rb");
            break;

        default:
            break;
    }
    if (nullptr == fp) {
        cout << "PLAY Audio File Open Failed!" << endl;
        return;
    }

    fread(&wav_header, 1, sizeof(wav_header), fp);
        
    play_set_pcm(fp, &wav_header);
}