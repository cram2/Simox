
#include "PlanningThread.h"

#include <ctime>
#include <iostream>

using namespace std;

namespace Saba
{

    //! constructor
    PlanningThread::PlanningThread(MotionPlannerPtr planner)
    {
        this->planner = planner;
        THROW_VR_EXCEPTION_IF(!planner, "NULL data");
        threadStarted = false;
        plannerFinished = false;
    }

    //! destructor
    PlanningThread::~PlanningThread()
    {
        stop();
    }

    void PlanningThread::start()
    {
        std::scoped_lock lock(mutex);

        if (threadStarted)
        {
            // thread already started, nothing to do
            return;
        }

        // since the start method is the only way to start the thread we can set the varaibles for indicating the state.
        threadStarted = true;
        plannerFinished = false;

        planningThread = std::thread([this]()
        {
            this->workingMethod();
        });
    }

    void PlanningThread::interrupt(bool waitUntilStopped)
    {
        if (!isRunning())
        {
            // thread not started, nothing to do
            return;
        }

        // this is not perfect: we are setting a bool without protecting it with a mutex. But it works...
        if (planner)
        {
            planner->stopExecution();
        }

        // todo: catch boost::thread_interrupted in MotionPlanners and be sure to call boost::threa::interrupt points during planning...
        //thread.interrupt();

        if (waitUntilStopped)
        {
            planningThread.join();
        }
    }


    void PlanningThread::stop()
    {
        interrupt(true);
    }

    bool PlanningThread::isRunning()
    {
        std::scoped_lock lock(mutex);
        return threadStarted;
    }

    MotionPlannerPtr PlanningThread::getPlanner()
    {
        return planner;
    }

    void PlanningThread::workingMethod()
    {
        if (!threadStarted)
        {
            VR_WARNING << "Thread should be in started mode?!" << std::endl;
        }

        VR_ASSERT(planner);

        bool res = planner->plan(true);

        mutex.lock();
        threadStarted = false;
        plannerFinished = res;
        mutex.unlock();
    }

}

