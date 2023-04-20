#ifndef __AUDITION_H__
#define __AUDITION_H__

#define FRAME_LEN           640     
#define BUFFER_SIZE         4096    
#define SAMPLE_RATE_16K     (16000) 
#define SAMPLE_RATE_8K      (8000)  
#define MAX_GRAMMERID_LEN   (32)    
#define MAX_PARAMS_LEN      (1024)  

void keywordDetection();
int awakeIsWakeUp();
void awakeSetWakeup(int state);

void asrStartRecord();
void asrFinishRecord();
int asrIsFinish();

void pcmCutRawAudio();
void pcmConvPCM2MP3();

void playReplyAudio(int audioCase);

#endif /* __AUDITION_H__ */