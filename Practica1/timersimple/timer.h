#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

class Timer
{
    public:

        Timer(){};

        template <class callable>        
        void connect(callable&& f)
        {
			std::thread([=]() 
            {
                while(true)
                {
                    elapsedTime.store(elapsedTime.load() + period.load());
					if(go.load())
						std::invoke(f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
                }
            }).detach();
        };
        
        void start(int p)
        {
			period.store(p);
			go.store(true);
        };
        
        void stop() { go.store(!go); };
		void setPeriod(int p) { period.store(p) ;};

        int getPeriod() { return period.load(); };
        int getElapsedTime() { return elapsedTime.load(); };
        
    private:
        std::atomic_bool go = false;
		std::atomic_int period = 0;
        std::atomic_int elapsedTime = 0;
    
};

#endif // TIMER_H
