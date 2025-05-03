#include "../include/ThreadManager.h"
#include <stdexcept>

ThreadManager::ThreadManager(size_t numThreads) 
    : numThreads(numThreads), running(true), activeThreads(0) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    // Start the thread manager immediately upon construction
    start();
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    std::lock_guard<std::mutex> lock(taskMutex);
    if (!running) {
        running = true;
    }
    
    // Clear any existing threads
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads.clear();
    
    // Create worker threads
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
    
    taskCondition.notify_all(); // Wake up all threads
    
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads.clear();
}

void ThreadManager::addTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        taskQueue.push(task);
    }
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
    
    bool wasRunning = isRunning();
    if (wasRunning) {
        stop();
    }
    
    numThreads = newNumThreads;
    
    if (wasRunning) {
        start();
    }
}

size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(taskMutex);
    taskCondition.wait(lock, [this]() {
        return taskQueue.empty() && activeThreads == 0;
    });
}

size_t ThreadManager::getActiveThreadCount() const {
    return activeThreads;
}

void ThreadManager::workerThread(size_t threadId) {
    while (running) {
        std::function<void()> task;
        
        {
            std::unique_lock<std::mutex> lock(taskMutex);
            taskCondition.wait_for(lock, std::chrono::milliseconds(1), [this]() {
                return !running || !taskQueue.empty();
            });
            
            if (!running && taskQueue.empty()) {
                return;
            }
            
            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
                ++activeThreads;
            }
        }
        
        if (task) {
            task();
            --activeThreads;
            taskCondition.notify_all();
        }
    }
}

void ThreadManager::processNextTask() {
    std::function<void()> task;
    
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (!taskQueue.empty()) {
            task = std::move(taskQueue.front());
            taskQueue.pop();
        }
    }
    
    if (task) {
        task();
    }
}