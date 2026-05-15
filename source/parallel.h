/*
 * Copyright (c) 2026 Chris Giles
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Chris Giles makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

#pragma once

// A small persistent thread pool used to parallelize the AVBD solver across CPU
// cores. Worker threads are created once and reused; parallelFor splits a range
// into contiguous chunks (one per worker plus the calling thread) so no thread is
// spawned per call. parallelFor must only be called from the owning thread.

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

class ThreadPool
{
public:
    // numThreads is the total lane count including the calling thread. A value of
    // 0 selects the hardware concurrency. 1 disables threading entirely.
    explicit ThreadPool(int numThreads);
    ~ThreadPool();

    // Total lanes (worker threads + calling thread).
    int laneCount() const { return (int)workers.size() + 1; }

    // Invokes fn(i) for i in [0, count), distributed across the lanes. Blocks
    // until every invocation has completed.
    void parallelFor(int count, const std::function<void(int)> &fn);

private:
    struct Worker
    {
        std::thread thread;
        std::mutex mutex;
        std::condition_variable cv;
        const std::function<void(int)> *fn = nullptr;
        int begin = 0;
        int end = 0;
        bool hasWork = false;
        bool done = true;
        bool stop = false;
    };

    std::vector<Worker *> workers;

    void workerLoop(Worker *w);
};
