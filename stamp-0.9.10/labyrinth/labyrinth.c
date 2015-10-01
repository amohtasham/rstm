/* =============================================================================
 *
 * labyrinth.c
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
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include "list.h"
#include "maze.h"
#include "router.h"
#include "thread.h"
#include "timer.h"
#include "types.h"
#include "controller_proxy.h"

#include <signal.h>
#include <time.h>

enum param_types {
    PARAM_BENDCOST = (unsigned char)'b',
    PARAM_THREAD   = (unsigned char)'t',
    PARAM_XCOST    = (unsigned char)'x',
    PARAM_YCOST    = (unsigned char)'y',
    PARAM_ZCOST    = (unsigned char)'z',
    PARAM_DURATION = (unsigned char)'d',
};

enum param_defaults {
    PARAM_DEFAULT_BENDCOST  = 1,
    PARAM_DEFAULT_THREAD    = 1,
    PARAM_DEFAULT_XCOST     = 1,
    PARAM_DEFAULT_YCOST     = 1,
    PARAM_DEFAULT_ZCOST     = 2,
    PARAM_DEFAULT_DURATION  = 0,
};


volatile bool_t   global_timedExecution    = FALSE;
volatile bool_t   global_isTerminated    = FALSE;
volatile bool_t   global_startExecution    = FALSE;

bool_t global_doPrint = FALSE;
char* global_inputFile = NULL;
long global_params[256]; /* 256 = ascii limit */


void signal_handler(int sig)
{
    switch (sig)
    {
        case SIGALRM:
            global_isTerminated = TRUE;
            break;
        case SIGUSR1:
            global_startExecution = TRUE;
            break;
    }
}


/* =============================================================================
 * displayUsage
 * =============================================================================
 */
static void
displayUsage (const char* appName)
{
    printf("Usage: %s [options]\n", appName);
    puts("\nOptions:                            (defaults)\n");
    printf("    b <INT>    [b]end cost          (%i)\n", PARAM_DEFAULT_BENDCOST);
    printf("    i <FILE>   [i]nput file name    (%s)\n", global_inputFile);
    printf("    p          [p]rint routed maze  (false)\n");
    printf("    t <UINT>   Number of [t]hreads  (%i)\n", PARAM_DEFAULT_THREAD);
    printf("    x <UINT>   [x] movement cost    (%i)\n", PARAM_DEFAULT_XCOST);
    printf("    y <UINT>   [y] movement cost    (%i)\n", PARAM_DEFAULT_YCOST);
    printf("    z <UINT>   [z] movement cost    (%i)\n", PARAM_DEFAULT_ZCOST);
    printf("    d <UINT>   [d]uration (secs)    (%i)\n", PARAM_DEFAULT_DURATION);

    exit(1);
}


/* =============================================================================
 * setDefaultParams
 * =============================================================================
 */
static void
setDefaultParams ()
{
    global_params[PARAM_BENDCOST] = PARAM_DEFAULT_BENDCOST;
    global_params[PARAM_THREAD]   = PARAM_DEFAULT_THREAD;
    global_params[PARAM_XCOST]    = PARAM_DEFAULT_XCOST;
    global_params[PARAM_YCOST]    = PARAM_DEFAULT_YCOST;
    global_params[PARAM_ZCOST]    = PARAM_DEFAULT_ZCOST;
    global_params[PARAM_DURATION]    = PARAM_DEFAULT_DURATION;
}


/* =============================================================================
 * parseArgs
 * =============================================================================
 */
static void
parseArgs (long argc, char* const argv[])
{
    long i;
    long opt;

    opterr = 0;

    setDefaultParams();

    while ((opt = getopt(argc, argv, "b:i:pt:x:y:z:d:")) != -1) {
        switch (opt) {
            case 'b':
            case 't':
            case 'x':
            case 'y':
            case 'z':
            case 'd':
                global_params[(unsigned char)opt] = atol(optarg);
                break;
            case 'i':
                global_inputFile = optarg;
                break;
            case 'p':
                global_doPrint = TRUE;
                break;
            case '?':
            default:
                opterr++;
                break;
        }
    }

    for (i = optind; i < argc; i++) {
        fprintf(stderr, "Non-option argument: %s\n", argv[i]);
        opterr++;
    }

    if (opterr) {
        displayUsage(argv[0]);
    }
}


/* =============================================================================
 * main
 * =============================================================================
 */
MAIN(argc, argv)
{
    /*
     * Initialization
     */
    parseArgs(argc, (char** const)argv);
    long numThread = global_params[PARAM_THREAD];
    SIM_GET_NUM_CPU(numThread);
    TM_STARTUP(numThread);
    P_MEMORY_STARTUP(numThread);
    thread_startup(numThread);
    maze_t* mazePtr = maze_alloc();
    assert(mazePtr);
    long numPathToRoute = maze_read(mazePtr, global_inputFile);
    router_t* routerPtr = router_alloc(global_params[PARAM_XCOST],
                                       global_params[PARAM_YCOST],
                                       global_params[PARAM_ZCOST],
                                       global_params[PARAM_BENDCOST]);
    assert(routerPtr);
    list_t* pathVectorListPtr = list_alloc(NULL);
    assert(pathVectorListPtr);

    long* numPathArray = (long *)SEQ_MALLOC(numThread * sizeof(long));
    assert(numPathArray);

    /*
     * Run transactions
     */
    router_solve_arg_t routerArg = {routerPtr, mazePtr, pathVectorListPtr, numPathArray};
    // NB: Since ASF/PTLSim "REAL" is native execution, and since we are using
    //     wallclock time, we want to be sure we read time inside the
    //     simulator, or else we report native cycles spent on the benchmark
    //     instead of simulator cycles.
    GOTO_SIM();
    long duration      = global_params[PARAM_DURATION];
    if (duration > 0)
    {
        global_timedExecution = TRUE;
        int coordinatorPID = 0;
        char *envVar = getenv("COORDINATOR_PID");
        if (envVar)
            coordinatorPID = atoi(envVar);
        int synchronizedExecution = 0;
        envVar = getenv("SYNCHRONIZED_EXECUTION");
        if (envVar)
            synchronizedExecution = atoi(envVar);
        if (synchronizedExecution && coordinatorPID)
        {
            printf("Synchronized execution is enabled.\n");
            global_startExecution = FALSE;
            signal(SIGUSR1, signal_handler);
            printf("I'm ready. Sending SIGUSR1 to the co-ordinator...\n");
            kill(coordinatorPID, SIGUSR1);//Letting the co-ordinator know that the process is ready for executing the workload
            printf("Waiting for the acknowledgement...\n");
            while (!global_startExecution) {
                /*waiting to the SIGUSR1 from the co-ordinator*/
                usleep(10);
            };
            printf("Executing the workload...\n");
        }
        controller_alloc(numThread);
        signal(SIGALRM, signal_handler);
        alarm(duration);
    }
    TIMER_T startTime;
    TIMER_READ(startTime);
#ifdef OTM
#pragma omp parallel
    {
        router_solve((void *)&routerArg);
    }
#else
    thread_start(router_solve, (void*)&routerArg);
#endif
    TIMER_T stopTime;
    TIMER_READ(stopTime);
    // NB: As above, timer reads must be done inside of the simulated region
    //     for PTLSim/ASF
    GOTO_REAL();

    long numPathRouted = 0;
    list_iter_t it;
    list_iter_reset(&it, pathVectorListPtr);
    while (list_iter_hasNext(&it, pathVectorListPtr)) {
        vector_t* pathVectorPtr = (vector_t*)list_iter_next(&it, pathVectorListPtr);
        numPathRouted += vector_getSize(pathVectorPtr);
    }
    printf("Paths routed    = %li\n", numPathRouted);
    printf("Elapsed time    = %f seconds\n", TIMER_DIFF_SECONDS(startTime, stopTime));
    if (duration > 0) {
        long numPath = 0;
        for (int i = 0 ; i < numThread ; i++) {
            numPath += numPathArray[i];
        }

        printf("Rate            = %0.1lf\n", (double)numPath/ TIMER_DIFF_SECONDS(startTime, stopTime));
    }
    /*
     * Check solution and clean up
     */
    assert(numPathRouted <= numPathToRoute);
    bool_t status = maze_checkPaths(mazePtr, pathVectorListPtr, global_doPrint);
    assert(status == TRUE);
    puts("Verification passed.");
    maze_free(mazePtr);
    router_free(routerPtr);
    controller_free();
    TM_SHUTDOWN();
    P_MEMORY_SHUTDOWN();

    thread_shutdown();


    MAIN_RETURN(0);
}


/* =============================================================================
 *
 * End of labyrinth.c
 *
 * =============================================================================
 */
