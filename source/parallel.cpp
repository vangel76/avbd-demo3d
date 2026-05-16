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

#include "parallel.h"
#include <chrono>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace
{
// Hint to the CPU that this is a spin-wait iteration (lets it save power and
// yields the pipeline to a sibling SMT thread). Falls back to a scheduler yield
// on non-x86 targets.
inline void cpuRelax()
{
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
    _mm_pause();
#elif defined(__i386__) || defined(__x86_64__)
    __builtin_ia32_pause();
#else
    std::this_thread::yield();
#endif
}

// Number of spin iterations before a thread gives up and sleeps. ~thousands of
// cpuRelax() calls cover a few hundred microseconds -- long enough to bridge the
// gaps between the solver's back-to-back parallelFor calls without sleeping,
// short enough that a truly idle pool sleeps almost immediately.
const int kSpinLimit = 8000;
} // namespace

ThreadPool::ThreadPool(int numThreads)
{
    if (numThreads <= 0)
        numThreads = (int)std::thread::hardware_concurrency();
    if (numThreads < 1)
        numThreads = 1;

    // workers excludes the calling thread, which is always lane 0.
    int workerCount = numThreads - 1;
    for (int i = 0; i < workerCount; ++i)
        workers.push_back(new Worker());
    for (Worker *w : workers)
        w->thread = std::thread(&ThreadPool::workerLoop, this, w);
}

ThreadPool::~ThreadPool()
{
    for (Worker *w : workers)
    {
        w->stop.store(true, std::memory_order_release);
        // Bump the generation so a spinning worker notices, and notify in case
        // it has fallen back to a CV sleep.
        w->workGen.fetch_add(1, std::memory_order_release);
        w->cv.notify_one();
    }
    for (Worker *w : workers)
    {
        if (w->thread.joinable())
            w->thread.join();
        delete w;
    }
}

void ThreadPool::workerLoop(Worker *w)
{
    unsigned lastGen = 0;
    for (;;)
    {
        // Wait for a new work generation. Spin first for low-latency pickup
        // during a tight solve loop; once kSpinLimit is reached, fall back to a
        // timed CV sleep so an idle pool does not keep a core busy. The 1 ms
        // timeout also makes a missed notify cost at most 1 ms (only ever on the
        // first dispatch after the pool goes idle).
        int spins = 0;
        while (w->workGen.load(std::memory_order_acquire) == lastGen)
        {
            if (w->stop.load(std::memory_order_acquire))
                return;
            if (++spins < kSpinLimit)
            {
                cpuRelax();
            }
            else
            {
                std::unique_lock<std::mutex> lk(w->mutex);
                w->cv.wait_for(lk, std::chrono::milliseconds(1), [&] {
                    return w->workGen.load(std::memory_order_acquire) != lastGen ||
                           w->stop.load(std::memory_order_acquire);
                });
            }
        }
        lastGen = w->workGen.load(std::memory_order_acquire);
        if (w->stop.load(std::memory_order_acquire))
            return;

        // fn/begin/end were published before the workGen release above, so the
        // acquire load makes them visible here.
        const std::function<void(int)> *fn = w->fn;
        int begin = w->begin;
        int end = w->end;
        for (int i = begin; i < end; ++i)
            (*fn)(i);

        w->done.store(true, std::memory_order_release);
    }
}

void ThreadPool::parallelFor(int count, const std::function<void(int)> &fn)
{
    if (count <= 0)
        return;

    int lanes = laneCount();
    if (lanes <= 1 || count < 2)
    {
        for (int i = 0; i < count; ++i)
            fn(i);
        return;
    }

    int chunk = (count + lanes - 1) / lanes;

    // Publish a chunk to each worker lane (1 .. workers). Writing fn/begin/end
    // before the release fetch_add makes them visible to the worker's acquire.
    for (size_t k = 0; k < workers.size(); ++k)
    {
        Worker *w = workers[k];
        int begin = (int)(k + 1) * chunk;
        int end = begin + chunk;
        if (begin > count)
            begin = count;
        if (end > count)
            end = count;

        w->fn = &fn;
        w->begin = begin;
        w->end = end;
        w->done.store(false, std::memory_order_relaxed);
        w->workGen.fetch_add(1, std::memory_order_release);
        w->cv.notify_one(); // only wakes the worker if it had fallen asleep
    }

    // The calling thread runs lane 0.
    int mainEnd = chunk < count ? chunk : count;
    for (int i = 0; i < mainEnd; ++i)
        fn(i);

    // Spin-wait for every worker lane to finish before fn goes out of scope.
    for (Worker *w : workers)
    {
        int spins = 0;
        while (!w->done.load(std::memory_order_acquire))
        {
            if (++spins < kSpinLimit)
                cpuRelax();
            else
                std::this_thread::yield();
        }
    }
}
