/*******************************************************************************
 * File: Queue.h
 *
 * Author: Alexy Torres Aurora Dugo
 *
 * Date: 13/08/2018
 *
 * Version: 1.0
 *
 * Thread safe blocking queue of the framework's core. These queues are based on
 * the modern C++ standard queues, and are just thread-safe custom wrappers. 
 * The policy used if FIFO.
 ******************************************************************************/
#ifndef __COMMON__QUEUE_H_
#define __COMMON__QUEUE_H_
#include <cstdint>   /* Generic types */
#include <queue>     /* std::queue */
#include <pthread.h> /* pthread_mutex_t, pthread_cond_t */
namespace nsCommon
{
    template<typename T>
    class Queue
    {
        private:
            /** The std queue containing our elements. */
            std::queue<T> queue;
            /** The queue lock. */
            pthread_mutex_t queueLock;
            /** The queue full synchronization condition variable. */
            pthread_cond_t queueFullSyncVar;
            /** The queue empty synchronization condition variable. */
            pthread_cond_t queueEmptySyncVar;
            /** Maximum size of the queue. */
            uint32_t maxSize;
        public:
            /**
             * @brief Initializes the queue and set its maximum size. If size is
             * set to 0, then the queue does not have any size limit.
             * 
             * @param maxSize The maximum size of the queue. If set to 0 the 
             * queue has no size limit.
             */
            Queue(const uint32_t maxSize)
            {
                this->maxSize = maxSize;
                /* Init synchronization primitives */
                this->queueLock         = PTHREAD_MUTEX_INITIALIZER;
                this->queueFullSyncVar  = PTHREAD_COND_INITIALIZER;
                this->queueEmptySyncVar = PTHREAD_COND_INITIALIZER;
            }
            /**
             * @brief Destroy the queue and all its contained elements.
             */
            ~Queue(void)
            {
            }
            /**
             * @brief Returns whether the queue is empty: i.e. whether its size 
             * is zero.
             * 
             * @returns true if the queue's size is 0, false 
             * otherwise.
             */
            bool empty(void) const
            {
                return this->queue.empty();
            }
            /**
             * @brief Returns the number of elements in the queue.
             * 
             * @returns The number of elements in the queue.
             */
            size_t size(void) const
            {
                return this->queue.size();
            }
            /**
             * @brief Inserts a new element at the end of the queue, after its 
             * current last element. The content of this new element is 
             * initialized to elem.
             * 
             * @param[in] elem Value to which the inserted element is 
             * initialized.
             */
            void push(const T& elem)
            {
                this->queue.push(elem);
            }
            /**
             * @brief Inserts a new element at the end of the queue, after its 
             * current last element. The content of this new element is 
             * initialized to elem.
             * 
             * @param[in] elem Value to which the inserted element is 
             * initialized.
             */
            void push(T&& elem)
            {
                /* Queue lock */
                pthread_mutex_lock((&this->queueLock));
                /* Wait until queue is filled */
                while(this->maxSize != 0 && 
                      this->queue.size() == this->maxSize)
                {
                    pthread_cond_wait(&(this->queueFullSyncVar), 
                                      &(this->queueLock));
                }
                /* Push the element */
                this->queue.push(elem);
                /* Signal that there is an element */
                pthread_cond_signal(&(this->queueEmptySyncVar));
                /* Queue unlock */
                pthread_mutex_unlock(&(this->queueLock));
            }
            /**
             * @brief Removes the next element in the queue and returns is, 
             * effectively reducing its size by one.
             * 
             * @return The the next element in the queue.
             */
            T pop(void)
            {
                /* Queue lock */
                pthread_mutex_lock(&(this->queueLock));
                /* Wait until queue is filled */
                while(this->queue.size() == 0)
                {
                    pthread_cond_wait(&(this->queueEmptySyncVar), 
                                      &(this->queueLock));
                }
                /* Get the value and remove it */
                T value = this->queue.front();
                this->queue.pop();
                /* Signal that there is an empty spot */
                pthread_cond_signal(&(this->queueFullSyncVar));
                /* Queue unlock */
                pthread_mutex_unlock(&(this->queueLock));
                return value;
            }
    };
}
#endif /* __COMMON__QUEUE_H_ */
