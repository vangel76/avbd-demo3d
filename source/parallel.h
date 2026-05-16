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
//
// The solver issues ~100 parallelFor calls per step (one per graph colour per
// iteration, plus the dual pass). To keep that dispatch cheap, worker threads
// hand off work through atomics and spin-wait for it; they only fall back to a
// condition-variable sleep once the pool has been idle for a while, so an idle
// pool (e.g. between viewport frames) does not burn CPU.

#include <atomic>
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

        // Work handoff. fn/begin/end are published by parallelFor and become
        // visible to the worker through the release/acquire on workGen.
        const std::function<void(int)> *fn = nullptr;
        int begin = 0;
        int end = 0;
        std::atomic<unsigned> workGen{0}; // bumped to publish a new work item
        std::atomic<bool> done{true};     // set by the worker when its chunk is done
        std::atomic<bool> stop{false};    // pool shutdown

        // Sleep fallback, used only when the worker has spun idle for a while.
        std::mutex mutex;
        std::condition_variable cv;
    };

    std::vector<Worker *> workers;

    void workerLoop(Worker *w);
};
