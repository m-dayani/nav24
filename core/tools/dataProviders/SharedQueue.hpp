//
// Created by root on 5/16/21.
//

#ifndef NAV24_SHAREDQUEUE_H
#define NAV24_SHAREDQUEUE_H

#include <string>
#include <vector>
#include <queue>
#include <memory>
#include <mutex>


namespace NAV24 {

    template<typename T>
    class SharedQueue {
    public:
        bool empty() {
            std::unique_lock<std::mutex> lock(mMutexBuffer);
            return mQueue.empty();
        }

        bool operator<(unsigned long nEls) {
            std::unique_lock<std::mutex> lock(mMutexBuffer);
            return mQueue.size() < nEls;
        }

        bool operator<=(unsigned long nEls) {
            std::unique_lock<std::mutex> lock(mMutexBuffer);
            return mQueue.size() <= nEls;
        }

        bool operator>(unsigned long nEls) {
            std::unique_lock<std::mutex> lock(mMutexBuffer);
            return mQueue.size() > nEls;
        }

        bool operator>=(unsigned long nEls) {
            std::unique_lock<std::mutex> lock(mMutexBuffer);
            return mQueue.size() >= nEls;
        }

        void fillBuffer(const std::vector<T>& vEls) {

            if (vEls.empty())
                return;

            std::unique_lock<std::mutex> lock1(mMutexBuffer);
            for (const T& ev : vEls) {
                mQueue.push(ev);
            }
        }

        unsigned long consumeBegin(unsigned long chunkSize, std::vector<T>& vEls) {

            std::unique_lock<std::mutex> lock1(mMutexBuffer);
            if (mQueue.empty())
                return 0;

            unsigned nEvs = std::min(chunkSize, mQueue.size());
            vEls.resize(nEvs);

            for (unsigned i = 0; i < nEvs; i++) {
                vEls[i] = mQueue.front();
                mQueue.pop();
            }
            return nEvs;
        }

        void push(T el) {
            std::unique_lock<std::mutex> lock1(mMutexBuffer);
            mQueue.push(el);
        }

        T front() {
            std::unique_lock<std::mutex> lock1(mMutexBuffer);
            return mQueue.front();
        }

        void pop() {
            std::unique_lock<std::mutex> lock1(mMutexBuffer);
            mQueue.pop();
        }

        size_t size() {
            std::unique_lock<std::mutex> lock1(mMutexBuffer);
            return mQueue.size();
        }

        void clear() {
            std::unique_lock<std::mutex> lock1(mMutexBuffer);

            while(!mQueue.empty()) {
                mQueue.pop();
            }
        }
    protected:
        std::queue<T> mQueue;
        std::mutex mMutexBuffer;
    };

} // NAV24


#endif //NAV24_SHAREDQUEUE_H
