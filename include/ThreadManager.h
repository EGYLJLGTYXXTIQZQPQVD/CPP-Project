#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional> // For std::function
#include <queue>      // For std::queue
#include <future>     // For std::packaged_task if returning results is needed (optional)
#include <numeric>    // For std::iota if needed

class ThreadManager
{
public:
    // Constructor: Initialize with a specific number of threads
    ThreadManager(size_t numThreads);
    // Destructor: Ensure threads are stopped and joined
    ~ThreadManager();

    // Delete copy constructor and assignment operator
    ThreadManager(const ThreadManager &) = delete;
    ThreadManager &operator=(const ThreadManager &) = delete;

    // Start the worker threads (call this after construction)
    // Note: Consider starting threads in the constructor directly.
    void start();

    // Signal threads to stop processing new tasks and finish current ones.
    // Waits for all threads to join.
    void stop();

    // Wait until the task queue is empty and all threads are idle.
    void waitForCompletion();

    // Add a task (function) to the queue for a worker thread to execute.
    // void addTask(std::function<void()> task); // Original signature

    // Template version to allow adding tasks that return values (using std::future)
    template <typename Func, typename... Args>
    auto addTask(Func &&func, Args &&...args) -> std::future<decltype(func(args...))>;

    // Get the number of tasks currently waiting in the queue.
    size_t getTaskCount() const;

    // Set the number of threads (can resize the pool, tricky to implement dynamically)
    // For simplicity in a competition, often resizing is not fully required/implemented.
    // We'll provide a basic placeholder or a stop/restart approach.
    void setNumThreads(size_t newNumThreads);
    size_t getNumThreads() const; // Get the target/current number of threads

    // Get the number of threads currently executing a task.
    size_t getActiveThreadCount() const;

    // Check if the ThreadManager is actively processing tasks.
    bool isRunning() const;

private:
    // The main function executed by each worker thread.
    void workerLoop(); // Renamed from workerThread for clarity

    std::vector<std::thread> workers;            // Stores the worker threads
    std::queue<std::function<void()>> taskQueue; // Queue of tasks (functions) to execute
    mutable std::mutex queueMutex;               // Mutex to protect access to the task queue
    std::condition_variable condition;           // Condition variable to signal new tasks or stop
    std::atomic<bool> stopSignal{false};         // Atomic flag to signal threads to stop
    std::atomic<size_t> busyThreads{0};          // Atomic counter for threads currently executing a task
    size_t threadCount;                          // Target number of threads

    // Condition variable and mutex for waitForCompletion synchronization
    mutable std::mutex completionMutex;
    std::condition_variable completionCondition;
};

// --- Template Implementation for addTask ---
// This needs to be in the header file because it's a template.
template <typename Func, typename... Args>
auto ThreadManager::addTask(Func &&func, Args &&...args) -> std::future<decltype(func(args...))>
{
    // Get the return type of the function
    using ReturnType = decltype(func(args...));

    // Create a packaged_task which wraps the function and its arguments
    // std::bind ensures arguments are correctly forwarded/copied/moved
    auto task_ptr = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<Func>(func), std::forward<Args>(args)...));

    // Get the future associated with the packaged_task
    std::future<ReturnType> future = task_ptr->get_future();

    // Lock the queue and add a lambda that will execute the packaged_task
    {
        std::unique_lock<std::mutex> lock(queueMutex);

        // Don't allow adding tasks if stop has been signaled
        if (stopSignal.load())
        {
            throw std::runtime_error("addTask called on stopped ThreadManager");
        }

        // Add the task (lambda) to the queue
        taskQueue.emplace([task_ptr]()
                          { (*task_ptr)(); });
    } // Unlock mutex

    // Notify one waiting worker thread that a task is available
    condition.notify_one();

    return future; // Return the future to the caller
}