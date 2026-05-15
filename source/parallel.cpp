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

ThreadPool::ThreadPool(int numThreads)
{
    if (numThreads <= 0)
        numThreads = (int)std::thread::hardware_concurrency();
    if (numThreads < 1)
        numThreads = 1;

    // workers excludes the calling thread, which is always lane 0.
    int workerCount = numThreads - 1;
    for (int i = 0; i < workerCount; ++i)
    {
        Worker *w = new Worker();
        workers.push_back(w);
    }
    for (Worker *w : workers)
        w->thread = std::thread(&ThreadPool::workerLoop, this, w);
}

ThreadPool::~ThreadPool()
{
    for (Worker *w : workers)
    {
        {
            std::lock_guard<std::mutex> lk(w->mutex);
            w->stop = true;
            w->hasWork = true;
        }
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
    for (;;)
    {
        const std::function<void(int)> *fn;
        int begin, end;
        {
            std::unique_lock<std::mutex> lk(w->mutex);
            w->cv.wait(lk, [w] { return w->hasWork; });
            if (w->stop)
                return;
            w->hasWork = false;
            fn = w->fn;
            begin = w->begin;
            end = w->end;
        }

        for (int i = begin; i < end; ++i)
            (*fn)(i);

        // Signal completion last; the calling thread may destroy fn afterwards,
        // but this worker only touches its own (pool-owned) state from here.
        {
            std::lock_guard<std::mutex> lk(w->mutex);
            w->done = true;
        }
        w->cv.notify_one();
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

    // Dispatch contiguous chunks to the worker lanes (1 .. workers).
    int dispatched = 0;
    for (size_t k = 0; k < workers.size(); ++k)
    {
        Worker *w = workers[k];
        int begin = (int)(k + 1) * chunk;
        int end = begin + chunk;
        if (begin > count)
            begin = count;
        if (end > count)
            end = count;
        {
            std::lock_guard<std::mutex> lk(w->mutex);
            w->fn = &fn;
            w->begin = begin;
            w->end = end;
            w->hasWork = true;
            w->done = false;
        }
        w->cv.notify_one();
        ++dispatched;
    }

    // The calling thread runs lane 0.
    int mainEnd = chunk < count ? chunk : count;
    for (int i = 0; i < mainEnd; ++i)
        fn(i);

    // Wait for every worker lane to finish before fn goes out of scope.
    for (int k = 0; k < dispatched; ++k)
    {
        Worker *w = workers[k];
        std::unique_lock<std::mutex> lk(w->mutex);
        w->cv.wait(lk, [w] { return w->done; });
    }
}
