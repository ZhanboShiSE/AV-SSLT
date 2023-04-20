#include <iostream>
#include <csignal>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "audition/audition.hpp"
#include "audition/message.hpp"

using std::cout;
using std::endl;

const int AWAKE_REPLY       = 0;
const int ASR_REPLY         = 1;


int main(int argc, char ** argv) {

    while (1) {
        cout << "waiting awaken..." << endl;

        keywordDetection();

        if (awakeIsWakeUp()) {
            pcmCutRawAudio();

            pcmConvPCM2MP3();

            msgAwakeRosPublish(argc, argv);

            // playReplyAudio(AWAKE_REPLY);

            clock_t start, finish;
            double total_time = 0.0;
            start = clock();

            asrStartRecord();

            while(!asrIsFinish()) {
                finish = clock();
                total_time = (double)(finish - start) / CLOCKS_PER_SEC;
                if (total_time > 5) {
                    cout << "time limited" << endl;
                    asrFinishRecord();
                    break;
                }
            }

            // playReplyAudio(ASR_REPLY);
        }

        awakeSetWakeup(false);
    }
}