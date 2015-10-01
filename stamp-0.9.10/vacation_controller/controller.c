/* =============================================================================
 *
 * client.c
 *
 * =============================================================================
 *
 * Copyright (C) Stanford University, 2006.  All Rights Reserved.
 * Author: Chi Cao Minh
 *
 * =============================================================================
 *
 * For the license of bayes/sort.h and bayes/sort.c, please see the header
 * of the files.
 *
 * ------------------------------------------------------------------------
 *
 * For the license of kmeans, please see kmeans/LICENSE.kmeans
 *
 * ------------------------------------------------------------------------
 *
 * For the license of ssca2, please see ssca2/COPYRIGHT
 *
 * ------------------------------------------------------------------------
 *
 * For the license of lib/mt19937ar.c and lib/mt19937ar.h, please see the
 * header of the files.
 *
 * ------------------------------------------------------------------------
 *
 * For the license of lib/rbtree.h and lib/rbtree.c, please see
 * lib/LEGALNOTICE.rbtree and lib/LICENSE.rbtree
 *
 * ------------------------------------------------------------------------
 *
 * Unless otherwise noted, the following license applies to STAMP files:
 *
 * Copyright (c) 2007, Stanford University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Stanford University nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY STANFORD UNIVERSITY ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL STANFORD UNIVERSITY BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * =============================================================================
 */


#include <assert.h>
#include <math.h>
#include "controller.h"
#include "thread.h"
#include "tm.h"

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

}

/* =============================================================================
 * controller_free
 * =============================================================================
 */

void controller_free()
{
    for (long i = 1 ; i < global_numThreads ; i++)
    {
        SEM_DESTROY(global_metadata[i].semaphore);
    }
    P_FREE(global_metadata);
    global_metadata = NULL;
    pthread_join(controllerThread, NULL);
}

/* =============================================================================
 * controller
 * -- Parallelism level controller
 * =============================================================================
 */
void *controller(void* args)
{
    int phase = 0;
    long i = 0;
    double currentRate = 0,bestSloweddownRate = 0;
    int mdPhases = 0;

    unsigned long currentTotalOperations = 0, prevTotalOperations = 0;

    double *rateHistory = (double *)P_MALLOC(sizeof(double) * global_numThreads);
    memset(rateHistory, 0, sizeof(double) * global_numThreads);

    double threadAvg = 0;

    int max = 0;

    struct timespec start, end, timer, dummyTime;

    int slowstart = 0;
    int slowdowns = 0;
    timer.tv_sec = 0;
    timer.tv_nsec = 5000000;//5 miliseconds
    int maxSlowdowns = 0;
    char *envVar = getenv("CONTROLLER_TIMER");
    if (envVar)
        //timer = atoi(envVar);
        timer.tv_nsec = atol(envVar) * 1000L;
    maxSlowdowns = 3;
    float dConstant = 2;
    envVar = getenv("MD_CONSTANT");
    if (envVar)
        dConstant = atof(envVar);

    envVar = getenv("MAX_SLOWDOWNS");
    if (envVar)
        maxSlowdowns = atoi(envVar);

    int controllerOutput = 1;
    envVar = getenv("CONTROLLER_OUTPUT");
    if (envVar)
        controllerOutput = atoi(envVar);

    clock_gettime(CLOCK_REALTIME, &start);
    prevTotalOperations = get_total_operations();
    int timerExtensions = 0;
    while (!global_isTerminated)
    {
        nanosleep(&timer, &dummyTime);

        clock_gettime(CLOCK_REALTIME, &end);
        currentTotalOperations = get_total_operations();
        currentRate =
                (double)(currentTotalOperations - prevTotalOperations)/((end.tv_sec - start.tv_sec ) * 1000L + ( end.tv_nsec - start.tv_nsec ) / 1000000L);
        if ((currentTotalOperations - prevTotalOperations) == 0)
        {
            timerExtensions ++;
            if (timerExtensions < 0)
                continue;
        }
        timerExtensions = 0;
        start = end;
        prevTotalOperations = currentTotalOperations;

        if (global_windowSize == 1 || currentRate > rateHistory[global_windowSize - 2])
        {
            if (phase == 0)
                rateHistory[global_windowSize - 1] = currentRate;
            phase = 0;
            slowdowns = 0;
            bestSloweddownRate = 0;
            if (global_windowSize < global_numThreads)
            {
//					if (slowstart)
//					{
//						slowstart = 0;
//						global_windowSize *= 2;
//						for (int j = 0 ; j < global_windowSize / 2 ; j++)
//							pthread_cond_signal(&count_threshold_cv);
//					}
//					else
//					{
                    global_windowSize++;
                    SEM_POST(global_metadata[((global_windowStart + global_windowSize - 1) % global_numThreads)].semaphore);
//					}
            }
        }
        else
        {
            if (phase == 0)
            {
                global_windowSize -= 1;
                phase = 1;
            }
            else
            {
                bestSloweddownRate = bestSloweddownRate < currentRate? currentRate: bestSloweddownRate;
                slowdowns++;
                if (slowdowns == maxSlowdowns)
                {
                    int newMax = floor((global_windowSize + 1)/dConstant);
                    if (newMax == global_windowSize)
                        newMax--;
                    memset(&rateHistory[newMax - 1], 0 , sizeof(double) * (global_numThreads - newMax + 1));
                    global_windowSize = newMax;
                    phase = 0;
                    slowdowns = 0;
                    bestSloweddownRate = 0;
                    mdPhases++;
                }
            }

        }
        if (controllerOutput)
            printf("%lu;%ld;%ld;%.2f \r\n", end.tv_sec * 1000000000L + end.tv_nsec, global_windowStart, global_windowSize, currentRate);
        i++;
        threadAvg = (threadAvg * (i-1) + global_windowSize)/(double)i;
        max = max >= global_windowSize? max : global_windowSize;
    }
    for (long i = global_windowSize ; i < global_numThreads ; i++)
        SEM_POST(global_metadata[i].semaphore);

    int process_output = 1;
    envVar = getenv("PROCESS_OUTPUT");
    if (envVar)
        process_output = atoi(envVar);
    if (process_output)
        printf("%.2f;%d\r\n", threadAvg, mdPhases);
    pthread_exit(NULL);
}


/* =============================================================================
 * before_task_acquisition
 * -- Waits until the thread is allowed to acquire a task
 * =============================================================================
 */
void before_task_acquisition(long threadId)
{
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
}

/* =============================================================================
 * after_task_acquisition
 * -- Increases the operations counter
 * =============================================================================
 */
void after_task_completion(long threadId)
{
    global_metadata[threadId].operations++;
}

/* =============================================================================
 * get_total_operations()
 * -- Returns the total number of finished operations (TXs, tasks, etc.)
 * =============================================================================
 */
unsigned long get_total_operations()
{
    unsigned long result = 0;

    for (long i = 0 ; i < global_numThreads ; i++)
    {
        result += global_metadata[i].operations;
    }
    return result;
}
/* =============================================================================
 *
 * End of controller.c
 *
 * =============================================================================
 */




