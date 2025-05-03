#include "../include/ThreadManager.h"
#include <stdexcept>

// Constructor: Initialize thread count, but don't start threads yet.
ThreadManager::ThreadManager(size_t numThreads)
    : threadCount(numThreads > 0 ? numThreads : std::thread::hardware_concurrency()) // Default to hardware cores if 0
{
    if (threadCount == 0)
        threadCount = 1; // Ensure at least one thread
    // Threads are started explicitly via start() method
}

// Destructor: Ensure stop() is called to clean up threads.
ThreadManager::~ThreadManager()
{
    // Check if stopSignal was already set. If not, call stop() to ensure proper cleanup.
    if (!stopSignal.load())
    {
        stop();
    }
}

// Start the worker threads.
void ThreadManager::start()
{
    if (!workers.empty())
    {
        // Optionally stop and restart, or just return. Let's just return for simplicity.
        return;
    }
    stopSignal.store(false); // Ensure stop signal is false
    busyThreads.store(0);    // Reset busy counter
    workers.reserve(threadCount);
    for (size_t i = 0; i < threadCount; ++i)
    {
        // Each thread executes the workerLoop method of this instance.
        workers.emplace_back(&ThreadManager::workerLoop, this);
    }
}

// Stop the ThreadManager.
void ThreadManager::stop()
{
    // Set the stop signal for all threads.
    stopSignal.store(true);

    // Notify all waiting threads to wake up and check the stop signal.
    condition.notify_all(); // Wake up all threads

    // Wait for each worker thread to finish its current task (if any) and exit.
    for (std::thread &worker : workers)
    {
        if (worker.joinable())
        {
            worker.join(); // Wait for the thread to complete
        }
    }
    workers.clear(); // Clear the vector of stopped threads

    // Clear any remaining tasks in the queue after stopping
    std::lock_guard<std::mutex> lock(queueMutex);
    std::queue<std::function<void()>> emptyQueue;
    std::swap(taskQueue, emptyQueue); // Efficiently clear the queue
}

// The main loop executed by each worker thread.
void ThreadManager::workerLoop()
{
    while (true)
    {                               // Loop indefinitely until stopSignal is true
        std::function<void()> task; // Variable to hold the task function

        // --- Critical Section: Wait for and retrieve a task ---
        {
            std::unique_lock<std::mutex> lock(queueMutex);

            // Wait condition: Stop if stopSignal is true OR if there's a task in the queue.
            condition.wait(lock, [this]
                           { return stopSignal.load() || !taskQueue.empty(); });

            // If stopSignal is true and the queue is empty, exit the loop.
            if (stopSignal.load() && taskQueue.empty())
            {
                return; // Exit the worker loop
            }

            // If there are tasks, get one.
            if (!taskQueue.empty())
            {
                task = std::move(taskQueue.front()); // Move task from queue
                taskQueue.pop();                     // Remove from queue
            }
            else
            {
                // Spurious wake-up or stop signal with non-empty queue: continue waiting/checking
                continue;
            }
        } // Mutex lock released here

        // --- Execute the Task ---
        if (task)
        {                  // Ensure we actually got a task
            busyThreads++; // Increment busy counter before executing
            try
            {
                task(); // Execute the retrieved task
            }
            catch (const std::exception &e)
            {
                // Handle exception in task
            }
            catch (...)
            {
                // Handle unknown exception in task
            }
            busyThreads--; // Decrement busy counter after execution

            // Notify threads waiting on waitForCompletion
            completionCondition.notify_all();
        }
    } // End of while(true) loop
}

// Wait for all tasks in the queue to be processed and all threads to become idle.
void ThreadManager::waitForCompletion()
{
    std::unique_lock<std::mutex> lock(completionMutex); // Use separate mutex for completion logic
    // Wait condition: Queue is empty AND no threads are currently busy executing tasks.
    completionCondition.wait(lock, [this]
                             {
        // Need to check taskQueue size under queueMutex as well for race conditions
        bool queueEmpty;
        {
             std::lock_guard<std::mutex> queueLock(queueMutex);
             queueEmpty = taskQueue.empty();
        }
        return queueEmpty && (busyThreads.load() == 0); });
}

// Get the number of tasks waiting in the queue.
size_t ThreadManager::getTaskCount() const
{
    std::lock_guard<std::mutex> lock(queueMutex); // Lock queue to read size safely
    return taskQueue.size();
}

// Basic implementation: Stops existing threads and starts new ones.
// NOTE: This is disruptive. A more advanced pool might resize dynamically.
void ThreadManager::setNumThreads(size_t newNumThreads)
{
    if (newNumThreads == 0)
        newNumThreads = 1; // Ensure at least one thread
    if (newNumThreads == threadCount && !workers.empty())
        return; // No change needed

    // Stop existing threads first
    if (!workers.empty())
    {
        stop(); // This clears the workers vector
    }

    // Update thread count and restart
    threadCount = newNumThreads;
    start(); // Start the new set of threads
}

// Get the target number of threads.
size_t ThreadManager::getNumThreads() const
{
    return threadCount;
}

// Get the number of threads currently executing tasks.
size_t ThreadManager::getActiveThreadCount() const
{
    return busyThreads.load(); // Read atomic counter
}

// Check if the manager has been stopped.
bool ThreadManager::isRunning() const
{
    // Considered "running" if stop hasn't been signaled AND threads exist.
    // This might need refinement based on exact definition needed.
    // Let's define running as "not stopped".
    return !stopSignal.load();
}