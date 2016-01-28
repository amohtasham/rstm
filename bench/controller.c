/* =============================================================================
 *
 * controller.c
 *
 * =============================================================================
 */

#include <assert.h>
#include <math.h>
#include <time.h>
#include "controller.h"
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "types.h"
#include "common/platform.hpp"

#define MALLOC malloc
#define FREE free
//#define CONTROLLER

const char *policies[9] = {"none","aimdp", "aimd", "aiad", "aimdpp", "aiadpp", "cubic","cubicp","f2c2"};
typedef enum {
    none = 0L,
    aimdp = 1L,
    aimd = 2L,
    aiad = 3L,
    aimdpp = 4L,
    aiadpp = 5L,
    cubicc = 6L,
    cubicp = 7L,
    f2c2 = 8L
} policy_t;

typedef struct {
    unsigned long currentTotalThroughput;
    unsigned long prevTotalThroughput;
    int phase;
    long iterations;
    double currentRate;
    int mdPhases;
    int avoidedMdPhases;
    int fairnessPhases;
    int slowdowns;
    float dConstant;
    int maxSlowdowns;
    unsigned long actualDelay;
    unsigned long processTime;

} controller_params_t;

typedef enum {
    linear = 0L,
    cubic = 1L,
    exponential = 2L

} change_mode_t;

void controller_aimd(controller_params_t &params);
void controller_aiad(controller_params_t &params);
void controller_aimdp(controller_params_t &params);
void controller_aimdpp(controller_params_t &params);
void controller_aiadpp(controller_params_t &params);
void controller_cubic(controller_params_t &params);
void controller_cubicp(controller_params_t &params);
void controller_f2c2(controller_params_t &params);

extern volatile bool_t   global_isTerminated;
extern volatile bool_t   global_timedExecution;

long global_windowStart;
long global_windowSize;
long global_numThreads;
long global_activeThreads;

metadata_t* global_metadata;
pthread_t controllerThread;



/* =============================================================================
 * controller_alloc
 * =============================================================================
 */

void controller_alloc(long numThreads)
{
#ifdef CONTROLLER
    posix_memalign((void **)&global_metadata, sizeof(metadata_t), sizeof(metadata_t) * numThreads);
    //semaphores = P_MALLOC(numThreads * sizeof(SEM_T));
    assert(global_metadata != NULL);
    for (long i = 0 ; i < numThreads ; i++)
    {
        SEM_INIT(global_metadata[i].semaphore, 0);
        global_metadata[i].operations = 0;
    }

    global_windowStart = 0;
    global_numThreads = numThreads;
    global_activeThreads = numThreads;

    pthread_attr_t controller_attr;
    struct sched_param param;
    param.__sched_priority = 99;
    pthread_attr_init(&controller_attr);
    pthread_attr_setschedpolicy(&controller_attr, SCHED_RR);
    pthread_attr_setschedparam(&controller_attr,&param);

    /* Initialize mutex and condition variable objects */
    //pthread_mutex_init(&count_mutex, NULL);
    //pthread_cond_init(&count_threshold_cv, NULL);
    pthread_create(&controllerThread, &controller_attr, &controller, NULL);
#endif
}
/* =============================================================================
 * controller_free
 * =============================================================================
 */

void controller_free()
{
#ifdef CONTROLLER
    pthread_join(controllerThread, NULL);
    for (long i = 1 ; i < global_numThreads ; i++)
    {
        SEM_DESTROY(global_metadata[i].semaphore);
    }
    FREE(global_metadata);
    global_metadata = NULL;
#endif
}

/* =============================================================================
 * controller
 * -- Parallelism level controller
 * =============================================================================
 */
void *controller(void* args)
{
    double threadAvg = 0;
    unsigned long  iterations = 0;

    int max = 0;
    controller_params_t params;

    struct timespec rtStart, rtEnd, processStart, processEnd, timer ;

    timer.tv_sec = 0;
    timer.tv_nsec = 10000000; //10 milliseconds
    char *envVar = getenv("CONTROLLER_TIMER");
    if (envVar)
        timer.tv_nsec = atol(envVar) * 1000000L; // CONTROLLER_TIMER is in milliseconds
    params.dConstant = 2;
    envVar = getenv("MD_CONSTANT");
    if (envVar)
        params.dConstant = atof(envVar);

    params.maxSlowdowns = 1;
    envVar = getenv("MAX_SLOWDOWNS");
    if (envVar)
        params.maxSlowdowns = atoi(envVar);

    int controllerOutput = 1;
    envVar = getenv("CONTROLLER_OUTPUT");
    if (envVar)
        controllerOutput = atoi(envVar);

    policy_t policy = cubicp;
    envVar = getenv("CONTROLLER_POLICY");
    if (envVar)
    {
        if (!strcasecmp(envVar, policies[aimd]))
            policy = aimd;
        else if (!strcasecmp(envVar, policies[aiad]))
            policy = aiad;
        else if (!strcasecmp(envVar, policies[aimdp]))
            policy = aimdp;
        else if (!strcasecmp(envVar, policies[aimdpp]))
            policy = aimdpp;
        else if (!strcasecmp(envVar, policies[aiadpp]))
            policy = aiadpp;
        else if (!strcasecmp(envVar, policies[cubicc]))
            policy = cubicc;
        else if (!strcasecmp(envVar, policies[cubicp]))
            policy = cubicp;
        else if (!strcasecmp(envVar, policies[f2c2]))
            policy = f2c2;
        else
            policy = none;
    }
    printf("\nPolicy: %s\n", policies[policy]);

    global_windowSize = (policy == none)? global_numThreads: 1;


    params.currentTotalThroughput = 0;
    params.prevTotalThroughput = 0;
    params.currentRate = 0;
    params.mdPhases = 0;
    params.avoidedMdPhases = 0;
    params.fairnessPhases = 0;
    params.slowdowns = 0;
    while (global_windowSize != global_activeThreads && !global_isTerminated)
    {
        //if (controllerOutput)
        //printf("wnd = %ld   #threads = %ld\r\n",global_windowSize,global_activeThreads);
        usleep(1);
    }
    while (!global_isTerminated && params.prevTotalThroughput == 0)
    {
        params.prevTotalThroughput = get_total_throughput();
    }

    //clock_gettime(CLOCK_REALTIME, &rtStart);
    //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &processStart);
    while (!global_isTerminated)
    {
        params.prevTotalThroughput = get_total_throughput();
        clock_gettime(CLOCK_REALTIME, &rtStart);
        nanosleep(&timer, NULL);
        clock_gettime(CLOCK_REALTIME, &rtEnd);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &processEnd);
        params.currentTotalThroughput = get_total_throughput();
        params.actualDelay = (rtEnd.tv_sec - rtStart.tv_sec ) * 1000L + ( rtEnd.tv_nsec - rtStart.tv_nsec ) / 1000000L;
        //params.processTime = (processEnd.tv_sec - processStart.tv_sec ) * 1000L + ( processEnd.tv_nsec - processStart.tv_nsec ) / 1000000L;
        params.currentRate = (double)(params.currentTotalThroughput - params.prevTotalThroughput) / params.actualDelay;
        //rtStart = rtEnd;
        //processStart = processEnd;
        //params.prevTotalThroughput = params.currentTotalThroughput;
        if (controllerOutput)
            printf("%lu;%ld;%ld;%.2f \r\n", rtEnd.tv_sec * 1000000000L + rtEnd.tv_nsec, global_windowStart, global_windowSize, params.currentRate);

        switch (policy) {
        case aimd:
            controller_aimd(params);
            break;
        case aimdp:
            controller_aimdp(params);
            break;
        case aiad:
            controller_aiad(params);
            break;
        case aimdpp:
            controller_aimdpp(params);
            break;
        case cubicc:
            controller_cubic(params);
            break;
        case cubicp:
            controller_cubicp(params);
            break;
        case f2c2:
            controller_f2c2(params);
            break;
        default:
            break;
        }

        iterations++;
        threadAvg = (threadAvg * (iterations-1) + global_windowSize)/(double)iterations;
        max = max >= global_windowSize? max : global_windowSize;
        while (global_windowSize != global_activeThreads && !global_isTerminated)
        {
            //if (controllerOutput)
            //printf("wnd = %ld   #threads = %ld\r\n",global_windowSize,global_activeThreads);
            usleep(1);
        };
    }
    //ublocking all threads
    long tid;
    for (tid = 0 ; tid < global_numThreads ; tid++)
        SEM_POST(global_metadata[tid].semaphore);

    int process_output = 1;
    envVar = getenv("PROCESS_OUTPUT");
    if (envVar)
        process_output = atoi(envVar);
    if (process_output)
    {
        printf("\r\nThread-count = %.2f\r\n", threadAvg);
        printf("Fairness Phases = %d\r\n", params.fairnessPhases);
        printf("MD-Phases = %d\r\n", params.mdPhases);
        printf("Avoided MD-Phases = %d\r\n", params.avoidedMdPhases);
    }
    pthread_exit(NULL);
}


void controller_aimd(controller_params_t &params)
{
    static long slowStart = 1;
    static double prevRate = 0;
    long newSize = global_windowSize;

    if (global_windowSize == 1 || params.currentRate > prevRate)
    {
        params.slowdowns = 0;
        if (slowStart)
        {
            newSize = fmin(global_windowSize * 2, global_numThreads);
        }
        else
        {
            newSize = fmin(global_windowSize + 1, global_numThreads);
        }
        while (global_windowSize < newSize)
        {
            global_windowSize++;
            SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
        }
        if (global_windowSize < global_numThreads)
        {
            prevRate = params.currentRate;
        }
    }
    else
    {
        params.slowdowns++;
        if (params.slowdowns == params.maxSlowdowns)
        {
            newSize = floor((global_windowSize + 1)/params.dConstant);
            if (newSize == global_windowSize)
                newSize--;
            prevRate = 0;
            global_windowSize = newSize;
            params.slowdowns = 0;
            slowStart = 0;
            params.mdPhases++;
        }
    }
}

void controller_aimdp(controller_params_t &params)
{
    static long slowStart = 1;
    static double prevRate = 0;
    static double prevPrevRate = 0;
    static long phase = 0;

    long newSize = global_windowSize;
    if (global_windowSize == 1 || params.currentRate > prevRate)
    {
        if (phase != 0)
        {
            params.avoidedMdPhases ++;
            phase = 0;
        }
        params.slowdowns = 0;
        if (slowStart)
        {
            newSize = fmin(global_windowSize * 2, global_numThreads);
        }
        else
        {
            newSize = fmin(global_windowSize + 1, global_numThreads);
        }
        if (global_windowSize < newSize)
        {
            prevPrevRate = prevRate;
            prevRate = params.currentRate;
            while (global_windowSize < newSize)
            {
                global_windowSize++;
                SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
            }
        }

    }
    else
    {
        if (phase == 0 && !slowStart)
        {
            global_windowSize -= 1;
            prevRate = prevPrevRate;
            phase = 1;
        }
        else
        {
            params.slowdowns++;
            if (params.slowdowns == params.maxSlowdowns)
            {
                newSize = fmax(fmin(floor((global_windowSize + 1)/params.dConstant), global_windowSize - 1),1);
                prevRate = 0;
                prevPrevRate = 0;
                global_windowSize = newSize;
                phase = 0;
                params.slowdowns = 0;
                params.mdPhases++;
                if (slowStart)
                    slowStart = 0;
            }
        }

    }
}

void controller_aimdpp(controller_params_t &params)
{
    static double prevRate = 0;
    static double prevPrevRate = 0;
    static long phase = 0;

    if ((global_windowSize > 1) && (params.actualDelay * (global_windowSize - 1) > params.processTime))
    {
        //the system is oversubscribed
        global_windowSize = (long)floor((double)params.processTime / (double)params.actualDelay);
        phase = 0;
        prevRate = 0;
        prevPrevRate = 0;
        phase = 0;
        params.slowdowns = 0;
    }
    else if (global_windowSize == 1 || params.currentRate > prevRate)
    {
        if (phase != 0)
            params.avoidedMdPhases ++;
        phase = 0;
        params.slowdowns = 0;
        if (global_windowSize < global_numThreads)
        {
            prevPrevRate = prevRate;
            prevRate = params.currentRate;

            global_windowSize++;
            SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
        }
    }
    else
    {
        if (phase == 0)
        {
            global_windowSize -= 1;
            prevRate = prevPrevRate;
            phase = 1;
        }
        else
        {
            params.slowdowns++;
            if (params.slowdowns == params.maxSlowdowns)
            {
                int newMax;
                newMax = floor((global_windowSize + 1)/params.dConstant);
                if (newMax == global_windowSize)
                    newMax--;
                prevRate = 0;
                prevPrevRate = 0;
                global_windowSize = newMax;
                phase = 0;
                params.slowdowns = 0;
                params.mdPhases++;
            }
        }

    }
}

void controller_aiad(controller_params_t &params)
{
    static double prevRate = 0;
    static long phase = 0;

    if (global_windowSize == 1 || params.currentRate > prevRate)
    {
        params.slowdowns = 0;
        if (global_windowSize < global_numThreads)
        {
            prevRate = params.currentRate;

            global_windowSize++;
            SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
        }
    }
    else
    {
        params.slowdowns++;
        if (params.slowdowns == params.maxSlowdowns)
        {
            int newMax;
            newMax = global_windowSize - 2 > 0 ? global_windowSize - 2 : 1;
            prevRate = 0;
            global_windowSize = newMax;
            phase = 0;
            params.slowdowns = 0;
        }
    }
}

void controller_aiadpp(controller_params_t &params)
{
    static double prevRate = 0;
    static double prevPrevRate = 0;
    static long phase = 0;


    if ((global_windowSize > 1) && (params.actualDelay * (global_windowSize - 1) > params.processTime))
    {
        //the system is oversubscribed
        global_windowSize = (long)floor((double)params.processTime / (double)params.actualDelay);
        phase = 0;
        prevRate = 0;
        prevPrevRate = 0;
        phase = 0;
        params.slowdowns = 0;
    }
    else if (global_windowSize == 1 || params.currentRate > prevRate)
    {
        params.slowdowns = 0;
        if (global_windowSize < global_numThreads)
        {
            prevRate = params.currentRate;

            global_windowSize++;
            SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
        }
    }
    else
    {
        params.slowdowns++;
        if (params.slowdowns == params.maxSlowdowns)
        {
            int newMax;
            newMax = global_windowSize - 2 > 0 ? global_windowSize - 2 : 1;
            prevRate = 0;
            global_windowSize = newMax;
            phase = 0;
            params.slowdowns = 0;
        }
    }
}

void controller_cubic(controller_params_t &params)
{
    static double b = 0.8;
    static double a = 3 * (1 - b) / (1 + b);
    static double c = 0.01;
    static long t_increase = 0;
    static double w_max = global_numThreads;
    static long slowStart = 1;
    static long cubicIncrease = 1;
    static double prevRate = 0;

    double w_tcp, w_cubic;
    long newSize = global_windowSize;
    if (global_windowSize == 1 || params.currentRate >= prevRate)
    {
        params.slowdowns = 0;
        if (global_windowSize < global_numThreads)
        {
            if (slowStart)
            {
                newSize = fmin(global_windowSize * 2, global_numThreads);
            }
            else if (cubicIncrease)
            {
                //printf("Cubic\n");
                t_increase++;
                w_tcp = (w_max * b) + (a * t_increase);
                w_cubic = c * pow(t_increase - pow(w_max * b / c, 1.0 / 3.0), 3) + w_max;
                newSize = (long)round(fmin(fmax(w_tcp, w_cubic), global_numThreads));
                newSize = (newSize == global_windowSize)||(newSize >= w_max)? global_windowSize + 1: newSize;
                if (newSize - global_windowSize > 1)
                {
                    cubicIncrease = 0;
                }
            }
            else if (!cubicIncrease)
            {
                //printf("NonCubic\n");
                newSize = fmin(global_windowSize + 1, global_numThreads);
                cubicIncrease = 1;
            }
            if (newSize > global_windowSize)
            {
                while (global_windowSize < newSize)
                {
                    global_windowSize++;
                    SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
                }
                prevRate = params.currentRate;
            }
        }
    }
    else
    {
        params.slowdowns++;
        if (params.slowdowns == params.maxSlowdowns)
        {
            slowStart = 0;
            w_max = global_windowSize;
            t_increase = 0;
            w_tcp = (w_max * b) + (a * t_increase);
            w_cubic = c * pow(t_increase - pow(w_max * b / c, 1.0 / 3.0), 3) + w_max;
            newSize = (long)round(fmin(fmax(w_tcp, w_cubic), global_numThreads));
            if (newSize == global_windowSize)
                newSize--;
            prevRate = 0;
            global_windowSize = newSize;
            params.slowdowns = 0;
            params.mdPhases++;
            cubicIncrease = 1;
        }
    }
}

//void controller_cubicp(controller_params_t &params)
//{
//    static double b = 0.7;
//    static double a = 3 * (1 - b) / (1 + b);
//    static double c = 1;
//    static long t_increase = 0;
//    static double w_max = global_numThreads;
//    static long slowStart = 1;
//    static long cubicIncrease = 1;
//    static double prevRate = 0;
//    static double prevPrevRate = 0;
//    static long phase = 0;


//    double w_tcp, w_cubic;
//    long newSize = global_windowSize;


//    if (global_windowSize == 1 || params.currentRate >= prevRate)
//    {
//        if (phase != 0)
//        {
//            params.avoidedMdPhases++;
//        }
//        params.slowdowns = 0;
//        if (global_windowSize < global_numThreads)
//        {
//            if (slowStart)
//            {
//                newSize = fmin(global_windowSize * 2, global_numThreads);
//            }
//            else if (phase > 0)
//            {
//                newSize = fmin(global_windowSize + 1, global_numThreads);
//                prevPrevRate = prevRate;
//                prevRate = params.currentRate;
//                cubicIncrease = 0;
//                phase--;
//            }
//            else if (cubicIncrease)
//            {
//                //printf("Cubic\n");
//                t_increase++;
//                w_tcp = (w_max * b) + (a * t_increase);
//                w_cubic = c * pow(t_increase - pow(w_max * b / c, 1.0 / 3.0), 3) + w_max;
//                newSize = (long)round(fmin(fmax(w_tcp, w_cubic), global_numThreads));
//                newSize = (newSize <= global_windowSize)||(newSize >= w_max)? global_windowSize + 1: newSize;
//                if (newSize - global_windowSize > 1)
//                {
//                    cubicIncrease = 0;
//                }
//            }
//            else if (!cubicIncrease)
//            {
//                //printf("NonCubic\n");
//                newSize = fmin(global_windowSize + 1, global_numThreads);
//                cubicIncrease = 1;
//            }
//            if (newSize > global_windowSize)
//            {
//                if (newSize - global_windowSize > 1)
//                    prevPrevRate = 0;
//                else
//                    prevPrevRate = prevRate;

//                prevRate = params.currentRate;
//                while (global_windowSize < newSize)
//                {
//                    global_windowSize++;
//                    SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
//                }
//            }
//        }
//        else if (slowStart == 1)
//        {
//            slowStart = 0;
//            prevRate = params.currentRate;
//        }
//        //prevRate = params.currentRate;
//    }
//    else
//    {
//        if (phase == 0 && !slowStart)
//        {
////            if (prevPrevRate == 0)
////            {
////                global_windowSize = fmax(1, global_windowSize - 2);
////                prevRate = 0;
////            }
////            else
////            {
////                global_windowSize -= 1;
////                prevRate = prevPrevRate;
////            }
////            cubicIncrease = 0;
////            phase = 1;
//            global_windowSize = fmax(1, global_windowSize - 2);
//            prevRate = 0;
//            cubicIncrease = 0;
//            phase = 2;
//        }
//        else
//        {
//            params.slowdowns++;
//            if (params.slowdowns == params.maxSlowdowns)
//            {
//                slowStart = 0;
//                w_max = global_windowSize;
//                t_increase = 0;
//                w_tcp = (w_max * b) + (a * t_increase);
//                w_cubic = c * pow(t_increase - pow(w_max * b / c, 1.0 / 3.0), 3) + w_max;
//                newSize = (long)round(fmin(fmax(w_tcp, w_cubic), global_numThreads));
//                if (newSize == global_windowSize)
//                    newSize--;
//                prevRate = 0;
//                prevPrevRate = 0;
//                global_windowSize = newSize;
//                params.slowdowns = 0;
//                phase = 0;
//                params.mdPhases++;
//                cubicIncrease = 0;
//            }
//        }
//    }
//}

void controller_cubicp(controller_params_t &params)
{
    static double b = 0.8;
    static double a = 3 * (1 - b) / (1 + b);
    static double c = 0.05;
    static long streak = 0;
    static double peak = 1;
    static change_mode_t growth_mode = cubic;
    static change_mode_t reduction_mode = linear;
    static double prevRate = 0;


    double w_tcp, w_cubic;
    long newSize = global_windowSize;


    if (global_windowSize == 1 || params.currentRate >= prevRate)
    {
        params.slowdowns = 0;
        if (growth_mode == exponential)
        {
            newSize = fmin(global_windowSize * 2, global_numThreads);
        }
        else if (growth_mode == linear)
        {
            newSize = fmin(global_windowSize + 1, global_numThreads);
            //if (newSize < peak)
            if (prevRate != 0)
                growth_mode = cubic;
        }
        else if (growth_mode == cubic)
        {
            streak++;
            w_tcp = (peak * b) + (a * streak);
            w_cubic = c * pow(streak - pow(peak * b / c, 1.0 / 3.0), 3) + peak;
            newSize = (long)round(fmin(fmax(fmax(w_tcp, w_cubic), global_windowSize + 1), global_numThreads));
            growth_mode = linear;
        }
        if (prevRate != 0)
            reduction_mode = linear;
        prevRate = params.currentRate;
        while (global_windowSize < newSize)
        {
            global_windowSize++;
            SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
        }
    }
    else
    {
        streak = 0;
        if (reduction_mode == linear)
        {
            newSize = fmax(1, global_windowSize - 2);
            reduction_mode = cubic;
        }
        else
        {
            peak = global_windowSize;
            w_tcp = (peak * b);
            w_cubic = c * pow(streak - pow(peak * b / c, 1.0 / 3.0), 3) + peak;
            newSize = (long)round(fmin(fmax(fmax(w_tcp, w_cubic),1), global_windowSize - 1));
            reduction_mode = linear;
            params.mdPhases++;
        }
        global_windowSize = newSize;
        growth_mode = linear;
        prevRate = 0;
    }
}

void controller_f2c2(controller_params_t &params)
{
    static long slowStart = 1;
    static long incMode = 1;
    static double prevRate = 0;
    static long phase = 0;


    long newSize = global_windowSize;

    if (global_windowSize == 1)
        incMode = 1;

    if (params.currentRate >= prevRate)
    {
        params.slowdowns = 0;
        if (slowStart)
        {
            newSize = fmin(global_windowSize * 2, global_numThreads);
        }
        else
        {
            newSize = fmax(fmin(global_windowSize + incMode, global_numThreads), 1);
        }
        prevRate = params.currentRate;
    }
    else
    {
        params.slowdowns++;
        if (params.slowdowns == params.maxSlowdowns)
        {
            if (slowStart)
            {
                slowStart = 0;
                newSize = fmin(global_windowSize / 2, global_numThreads);
                incMode = 1;
            }
            else
            {
                incMode *= -1;
                newSize = fmax(fmin(global_windowSize + incMode, global_numThreads), 1);
            }
            prevRate = params.currentRate;
            phase = 0;
            params.slowdowns = 0;
        }
    }
    if (global_windowSize < newSize)
    {
        while (global_windowSize < newSize)
        {
            global_windowSize++;
            SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
        }
    }
    else
    {
        global_windowSize = newSize;
    }
}


/* =============================================================================
 * wait_for_turn
 * -- Waits until the thread is allowed to acquire a task
 * =============================================================================
 */
void wait_for_turn(long threadId)
{
#ifdef CONTROLLER
    long windowStart = global_windowStart;
    long windowSize = global_windowSize;
    long windowEnd = (windowStart + windowSize - 1) % global_numThreads;
    if ((windowEnd >= windowStart && (threadId < windowStart || threadId > windowEnd)) ||
            (windowEnd < windowStart && threadId < windowStart && threadId > windowEnd))
    {
        __sync_fetch_and_sub(&global_activeThreads, 1);
        SEM_WAIT(global_metadata[threadId].semaphore);
        __sync_fetch_and_add(&global_activeThreads, 1);
        //			pthread_mutex_lock(&count_mutex);
        //			if (thread_count > global_windowSize)
        //			{
        //				thread_count--;
        //				pthread_cond_wait(&count_threshold_cv, &count_mutex);
        //				thread_count++;
        //			}
        //			pthread_mutex_unlock(&count_mutex);
    }
#endif
}

/* =============================================================================
 * add_to_completed_tasks
 * -- Increases the operations counter
 * =============================================================================
 */
void add_throughput(long threadId, ulong_t count)
{
#ifdef CONTROLLER
    global_metadata[threadId].operations += count;
#endif
}

/* =============================================================================
 * get_total_operations()
 * -- Returns the total number of finished operations (TXs, tasks, etc.)
 * =============================================================================
 */
ulong_t get_total_throughput()
{
    ulong_t result = 0;
    long i;
    for (i = 0 ; i < global_numThreads ; i++)
    {
        result += global_metadata[i].operations;
    }
    return result;
}


/* =============================================================================
 * get_thread_time_micro
 * -- Gets the current thread's processing time in micro-seconds
 * =============================================================================
 */
ulong_t get_thread_time()
{
    //return tick();
#if CONTROLLER
    struct timespec timeSpec;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timeSpec);
    return (timeSpec.tv_sec * 1000000L) + (timeSpec.tv_nsec / 1000L);
#else
    struct timespec timeSpec;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timeSpec);
    return (timeSpec.tv_sec * 1000000L) + (timeSpec.tv_nsec / 1000L);
#endif
}


/* =============================================================================
 *
 * End of controller.c
 *
 * =============================================================================
 */





