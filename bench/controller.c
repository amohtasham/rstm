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
#include "types.h"

#define MALLOC malloc
#define FREE free

const char *policies[3] = {"aimdp", "aimd", "aiad"};
typedef enum {
    aimdp = 0L,
    aimd = 1L,
    aiad = 2L
} policy_t;

extern volatile bool_t   global_isTerminated;
extern volatile bool_t   global_timedExecution;

long global_windowStart;
long global_windowSize;
long global_numThreads;

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
    global_windowSize = 1;
    global_numThreads = numThreads;

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
    int phase = 0;
    long iterations = 0;
    double currentRate = 0;
    double prevRate = 0;
    double prevPrevRate = 0;
    double bestSloweddownRate = 0;
    int mdPhases = 0;
    int avoidedMdPhases = 0;
    int fairnessPhases = 0;

    unsigned long currentTotalThroughput = 0;
    unsigned long prevTotalThroughput = 0;

    //double *rateHistory = (double *)MALLOC(sizeof(double) * global_numThreads);
    //ulong *timeHistory = (ulong *)MALLOC(sizeof(ulong) * global_numThreads);
    //memset(rateHistory, 0, sizeof(double) * global_numThreads);
    //memset(timeHistory, 0, sizeof(ulong) * global_numThreads);

    double threadAvg = 0;

    int max = 0;

    struct timespec rtStart, rtEnd, processStart, timer ;

    int slowdowns = 0;
    timer.tv_sec = 0;
    timer.tv_nsec = 10000000;
    char *envVar = getenv("CONTROLLER_TIMER");
    if (envVar)
        timer.tv_nsec = atol(envVar) * 1000000L;
    float dConstant = 2;
    envVar = getenv("MD_CONSTANT");
    if (envVar)
        dConstant = atof(envVar);

    int maxSlowdowns = 3;
    envVar = getenv("MAX_SLOWDOWNS");
    if (envVar)
        maxSlowdowns = atoi(envVar);

    int controllerOutput = 1;
    envVar = getenv("CONTROLLER_OUTPUT");
    if (envVar)
        controllerOutput = atoi(envVar);

    policy_t policy = aimdp;
    envVar = getenv("CONTROLLER_POLICY");
    if (envVar)
    {
        if (!strcasecmp(envVar, policies[aimd]))
            policy = aimd;
        else if (!strcasecmp(envVar, policies[aiad]))
            policy = aiad;
    }
    printf("\nPolicy: %s\n", policies[policy]);
    while (!global_isTerminated && prevTotalThroughput == 0)
    {
        prevTotalThroughput = get_total_throughput();
    }
    int timerExtensions = 0;
    clock_gettime(CLOCK_REALTIME, &rtStart);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &processStart);
    while (!global_isTerminated)
    {
        nanosleep(&timer, NULL);
        clock_gettime(CLOCK_REALTIME, &rtEnd);
        currentTotalThroughput = get_total_throughput();
        long actualDelay = (rtEnd.tv_sec - rtStart.tv_sec ) * 1000L + ( rtEnd.tv_nsec - rtStart.tv_nsec ) / 1000000L;
        currentRate = (double)(currentTotalThroughput - prevTotalThroughput) / actualDelay;
        if ((currentTotalThroughput - prevTotalThroughput) == 0)
        {
            timerExtensions ++;
            if (timerExtensions < 0)
                continue;
        }
        timerExtensions = 0;
        rtStart = rtEnd;
        prevTotalThroughput = currentTotalThroughput;

//        if (global_windowSize == 1 || currentRate > rateHistory[global_windowSize - 2])
        if (global_windowSize == 1 || currentRate > prevRate)
        {
            if (phase == 0)
            {
                //rateHistory[global_windowSize - 1] = currentRate;
                //prevPrevRate = prevRate;
                //prevRate = currentRate;
            }
            else
                avoidedMdPhases ++;
            phase = 0;
            slowdowns = 0;
            bestSloweddownRate = 0;
            if (global_windowSize < global_numThreads)
            {
                prevPrevRate = prevRate;
                prevRate = currentRate;

                global_windowSize++;
                SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
            }
        }
        else
        {
            if (phase == 0 && policy == aimdp)
            {
                global_windowSize -= 1;
                prevRate = prevPrevRate;
                phase = 1;
            }
            else
            {
                bestSloweddownRate = bestSloweddownRate < currentRate? currentRate: bestSloweddownRate;
                slowdowns++;
                if (slowdowns == maxSlowdowns)
                {
                    int newMax;
                    if (policy == aimd || policy == aimdp)
                    {
//                        newMax = floor((global_windowSize + 1)/dConstant);
//                        if (newMax == global_windowSize)
//                            newMax--;
//                        memset(&rateHistory[newMax - 1], 0 , sizeof(double) * (global_numThreads - newMax + 1));
                        newMax = floor((global_windowSize + 1)/dConstant);
                        if (newMax == global_windowSize)
                            newMax--;
                        prevRate = 0;
                        prevPrevRate = 0;

                    }
                    else if (policy == aiad)
                    {
//                        newMax = global_windowSize - 2 > 0 ? global_windowSize - 2 : 1;
//                        memset(&rateHistory[newMax - 1], 0 , sizeof(double) * (global_numThreads - newMax + 1));
                        newMax = global_windowSize - 2 > 0 ? global_windowSize - 2 : 1;
                        prevRate = 0;
                        prevPrevRate = 0;
                    }
                    global_windowSize = newMax;
                    phase = 0;
                    slowdowns = 0;
                    bestSloweddownRate = 0;
                    mdPhases++;
                }
            }

        }
        if (controllerOutput)
            printf("%lu;%ld;%ld;%.2f \r\n", rtEnd.tv_sec * 1000000000L + rtEnd.tv_nsec, global_windowStart, global_windowSize, currentRate);
        iterations++;
        threadAvg = (threadAvg * (iterations-1) + global_windowSize)/(double)iterations;
        max = max >= global_windowSize? max : global_windowSize;
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
        printf("Fairness Phases = %d\r\n", fairnessPhases);
        printf("MD-Phases = %d\r\n", mdPhases);
        printf("Avoided MD-Phases = %d\r\n", avoidedMdPhases);
    }
    pthread_exit(NULL);
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
        SEM_WAIT(global_metadata[threadId].semaphore);
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
void add_throughput(long threadId, long count)
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
unsigned long get_total_throughput()
{
    unsigned long result = 0;
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
ulong_t get_thread_time_micro()
{
#if CONTROLLER
    //return tick();
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





