/* =============================================================================
 *
 * controller.h
 *
 * =============================================================================
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H 1

#include <semaphore.h>

#include "types.h"

#define SEM_T  sem_t
#define SEM_INIT(sem, value)  sem_init(&sem, 0, value)
#define SEM_DESTROY(sem)  sem_destroy(&sem)
#define SEM_POST(sem)  sem_post(&sem)
#define SEM_WAIT(sem)  sem_wait(&sem)

//64-byte = cache-line size
typedef struct
{
    SEM_T semaphore;
    ulong_t operations;
    long _pad[3];
} metadata_t;


/* =============================================================================
 * controller_alloc
 * =============================================================================
 */
void controller_alloc(long numThreads);


/* =============================================================================
 * controller_free
 * =============================================================================
 */
void controller_free();

/* =============================================================================
 * controller
 * -- Parallelism level controller
 * =============================================================================
 */
void *controller(void* args);

/* =============================================================================
 * wait_for_turn
 * -- Waits until the thread is allowed to acquire a task
 * =============================================================================
 */
void wait_for_turn(long threadId);

/* =============================================================================
 * add_operations
 * -- Increases the operations counter
 * =============================================================================
 */
void add_throughput(long threadId, long count);

/* =============================================================================
 * get_total_operations()
 * -- Returns the total number of finished operations (TXs, tasks, etc.)
 * =============================================================================
 */
ulong_t get_total_throughput();


/* =============================================================================
 * controller
 * -- Parallelism level controller
 * =============================================================================
 */
void *controller(void* args);


/* =============================================================================
 * controller
 * -- Gets the current thread's processing time in micro-seconds
 * =============================================================================
 */
ulong_t get_thread_time_micro();


#endif /* CONTROLLER_H */


/* =============================================================================
 *
 * End of controller.h
 *
 * =============================================================================
 */
