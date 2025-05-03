#include "../include/ThreadManager.h"
#include <stdexcept>
#include <algorithm>

ThreadManager::ThreadManager(size_t numThreads)
    : numThreads(numThreads), running(false) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    // Initialize thread load counters
    threadLoads.resize(numThreads);
    for (auto& load : threadLoads) {
        load = 0;
    }
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    std::lock_guard<std::mutex> lock(taskMutex);
    if (running) return;
    
    running = true;
    threads.clear();
    
    // Create and start worker threads
    for (size_t i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadManager::workerThread, this, i);
    }
}

void ThreadManager::stop() {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (!running) return;
        running = false;
    }
    
    // Notify all threads to check the running flag
    taskCondition.notify_all();
    
    // Join all threads
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    threads.clear();
}

void ThreadManager::workerThread(size_t threadId) {
    while (true) {
        std::function<void()> task;
        
        {
            std::unique_lock<std::mutex> lock(taskMutex);
            
            taskCondition.wait(lock, [this] {
                return !running || !taskQueue.empty();
            });
            
            // Exit if thread manager is stopped and no tasks remain
            if (!running && taskQueue.empty()) {
                break;
            }
            
            // Get next task
            if (!taskQueue.empty()) {
                task = taskQueue.front();
                taskQueue.pop();
            } else {
                continue; // No tasks, but still running
            }
        }
        
        // Execute the task
        if (task) {
            activeThreads++;
            threadLoads[threadId]++;
            
            try {
                task();
            } catch (...) {
                // Prevent exceptions from killing the thread
            }
            
            activeThreads--;
        }
    }
}

void ThreadManager::addTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (!running) {
            throw std::runtime_error("Thread manager is not running");
        }
        taskQueue.push(task);
    }
    
    // Notify one thread that a task is available
    taskCondition.notify_one();
}

bool ThreadManager::isRunning() const {
    return running;
}

size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

void ThreadManager::setNumThreads(size_t newNumThreads) {
    if (newNumThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    // Stop the thread manager if it's running
    bool wasRunning = running;
    if (wasRunning) {
        stop();
    }
    
    numThreads = newNumThreads;
    
    // Resize thread loads vector
    threadLoads.resize(numThreads);
    for (auto& load : threadLoads) {
        load = 0;
    }
    
    // Restart if it was running
    if (wasRunning) {
        start();
    }
}

size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion() {
    while (true) {
        std::unique_lock<std::mutex> lock(completionMutex);
        
        // Check if there are no active threads and no pending tasks
        if (activeThreads == 0) {
            std::lock_guard<std::mutex> taskLock(taskMutex);
            if (taskQueue.empty()) {
                break;
            }
        }
        
        // Wait a bit before checking again
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void ThreadManager::processNextTask() {
    std::function<void()> task;
    
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (taskQueue.empty()) {
            return;
        }
        
        task = taskQueue.front();
        taskQueue.pop();
    }
    
    if (task) {
        task();
    }
}

size_t ThreadManager::getActiveThreadCount() const {
    return activeThreads;
}