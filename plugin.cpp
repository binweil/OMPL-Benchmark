#include <functional>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <string>

#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
//#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/TRRT.h>

#include "v_repPlusPlus/Plugin.h"
#include "plugin.h"
#include "stubs.h"
#include "sqlite3.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
size_t collision_count;

struct LuaCallbackFunction
{
    // name of the Lua function
    std::string function;
    // id of the V-REP script where the function is defined in
    simInt scriptId;
};

struct ObjectDefHeader
{
    // internal handle of this object (used by the plugin):
    simInt handle;
    // name of this object:
    std::string name;
    // objects created during simulation will be destroyed when simulation terminates:
    bool destroyAfterSimulationStop;
};

struct StateSpaceDef
{
    ObjectDefHeader header;
    // type of this state space:
    StateSpaceType type;
    // V-REP handle of the object (objects, or joint if type = joint_position):
    simInt objectHandle;
    // object handle in order to specify optional reference frame that is not absolute
    // for sim_ompl_statespace_pose2d, etc.
    simInt refFrameHandle;
    // weight of this state space component (used for state distance calculation):
    simFloat weight;
    // lower bounds of search space:
    std::vector<simFloat> boundsLow;
    // upper bounds of search space:
    std::vector<simFloat> boundsHigh;
    // use this state space as the default projection:
    bool defaultProjection;
    // (specific to dubins state space) turning radius:
    double dubinsTurningRadius;
    // (specific to dubins state space) symmetric:
    bool dubinsIsSymmetric;
};

struct TaskDef
{
    ObjectDefHeader header;
    // state space is a composition of elementary state spaces (internal handles to StateSpaceDef objects):
    std::vector<simInt> stateSpaces;
    // handle of the collision pairs:
    std::vector<simInt> collisionPairHandles;
    // start state:
    std::vector<simFloat> startState;
    // goal can be specified in different ways:
    struct Goal
    {
        enum {STATE, DUMMY_PAIR, CLLBACK} type;
        // goal ref. dummy:
        int refDummy;
        // goal metric:
        float metric[4]; // x,y,z,angle(orientation), relative to refDummy
        // goal tolerance:
        float tolerance;
        // goal state:
        std::vector<std::vector<simFloat> > states;
        // goal dummy pair:
        struct {simInt goalDummy, robotDummy;} dummyPair;
        // goal callback:
        LuaCallbackFunction callback;
    } goal;
    // state validation:
    struct StateValidation
    {
        enum {DEFAULT, CLLBACK} type;
        // state validation callback:
        LuaCallbackFunction callback;
    } stateValidation;
    // resolution at which state validity needs to be verified in order for a
    // motion between two states to be considered valid (specified as a
    // fraction of the space's extent)
    float stateValidityCheckingResolution;
    // state sampling:
    struct ValidStateSampling
    {
        enum {DEFAULT, CLLBACK} type;
        // state sampling callback:
        LuaCallbackFunction callback;
        // "near" state sampling callback:
        LuaCallbackFunction callbackNear;
    } validStateSampling;
    // projection evaluation:
    struct ProjectionEvaluation
    {
        enum {DEFAULT, CLLBACK} type;
        // projection evaluation callback:
        LuaCallbackFunction callback;
        // size of the projection (for callback)
        int dim;
    } projectionEvaluation;
    // search algorithm to use:
    Algorithm algorithm;
    // state space dimension:
    int dim;
    // how many things we should say in the V-REP console? (0 = stay silent)
    int verboseLevel;
    // OMPL classes (created with the setup() command):
    // state space
    ob::StateSpacePtr stateSpacePtr;
    // space information
    ob::SpaceInformationPtr spaceInformationPtr;
    // projection evaluator object
    ob::ProjectionEvaluatorPtr projectionEvaluatorPtr;
    // problem definition
    ob::ProblemDefinitionPtr problemDefinitionPtr;
    // planner
    ob::PlannerPtr planner;
};

std::map<simInt, TaskDef *> tasks;
std::map<simInt, StateSpaceDef *> statespaces;
simInt nextTaskHandle = 1000;
simInt nextStateSpaceHandle = 9000;

// this function will be called at simulation end to destroy objects that
// were created during simulation, which otherwise would leak indefinitely:
template<typename T>
void destroyTransientObjects(std::map<simInt, T *>& c)
{
    std::vector<simInt> transientObjects;

    for(typename std::map<simInt, T *>::const_iterator it = c.begin(); it != c.end(); ++it)
    {
        if(it->second->header.destroyAfterSimulationStop)
            transientObjects.push_back(it->first);
    }

    for(size_t i = 0; i < transientObjects.size(); i++)
    {
        simInt key = transientObjects[i];
        T *t = c[key];
        c.erase(key);
        delete t;
    }
}

void destroyTransientObjects()
{
    destroyTransientObjects(tasks);
    destroyTransientObjects(statespaces);
}

class ProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    ProjectionEvaluator(const ob::StateSpacePtr& space, TaskDef *task)
        : ob::ProjectionEvaluator(space), statespace(space)
    {
        this->task = task;

        dim = 0;

        switch(task->projectionEvaluation.type)
        {
        case TaskDef::ProjectionEvaluation::DEFAULT:
            switch(task->goal.type)
            {
            case TaskDef::Goal::STATE:
            case TaskDef::Goal::CLLBACK:
                dim = defaultProjectionSize();
                break;
            case TaskDef::Goal::DUMMY_PAIR:
                dim = dummyPairProjectionSize();
                break;
            default:
                // this will never happen
                dim = 0;
                break;
            }
            break;
        case TaskDef::ProjectionEvaluation::CLLBACK:
            dim = luaProjectCallbackSize();
            break;
        default:
            // this will never happen
            dim = 0;
            break;
        }
    }

    virtual unsigned int getDimension(void) const
    {
        return dim;
    }

    virtual void defaultCellSizes(void)
    {
        // TODO: handle this in the plugin API
        cellSizes_.resize(dim);
        for(int i = 0; i < dim; i++)
            cellSizes_[i] = 0.05;
    }

    virtual void project(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        for(int i = 0; i < dim; i++)
            projection(i) = 0.0;

        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        switch(task->projectionEvaluation.type)
        {
        case TaskDef::ProjectionEvaluation::DEFAULT:
            switch(task->goal.type)
            {
            case TaskDef::Goal::STATE:
            case TaskDef::Goal::CLLBACK:
                defaultProjection(state, projection);
                break;
            case TaskDef::Goal::DUMMY_PAIR:
                dummyPairProjection(state, projection);
                break;
            }
            break;
        case TaskDef::ProjectionEvaluation::CLLBACK:
            luaProjectCallback(state, projection);
            break;
        }
    }

protected:
    virtual int defaultProjectionSize() const
    {
        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            if(!stateSpace->defaultProjection) continue;

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
            case sim_ompl_statespacetype_position2d:
                return 2;
            case sim_ompl_statespacetype_pose3d:
            case sim_ompl_statespacetype_position3d:
                return 3;
            case sim_ompl_statespacetype_joint_position:
                return 1;
            case sim_ompl_statespacetype_dubins:
                return 2;
            }
        }

        return 0;
    }

    virtual void defaultProjection(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            if(!stateSpace->defaultProjection) continue;

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                break;
            case sim_ompl_statespacetype_pose3d:
                projection(0) = s->as<ob::SE3StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE3StateSpace::StateType>(i)->getY();
                projection(2) = s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                break;
            case sim_ompl_statespacetype_position2d:
                projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                break;
            case sim_ompl_statespacetype_position3d:
                projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                projection(2) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                break;
            case sim_ompl_statespacetype_joint_position:
                projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                break;
            case sim_ompl_statespacetype_dubins:
                projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                break;
            }

            break;
        }
    }

    virtual int dummyPairProjectionSize() const
    {
        /*
        return 3;
        */
        int s = 0;
        for(int i = 0; i < 3; i++)
        {
            if(task->goal.metric[i] != 0.0)
                s++;
        }
        if(s == 0)
            s = 1; // if X/Y/Z are ignored
        return s;
    }

    virtual void dummyPairProjection(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        /*
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, -1, &pos[0]);
        projection(0) = pos[0];
        projection(1) = pos[1];
        projection(2) = pos[2];
        */

        // TODO: don't we need to apply the provided state to the robot, read the tip dummy's position, then project it?

        // do projection, only for axis that should not be ignored:
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, task->goal.refDummy, &pos[0]);
        int ind = 0;
        for(int i = 0; i < 3; i++)
        {
            if(task->goal.metric[i] != 0.0)
                projection(ind++) = pos[i];
        }
        if(ind == 0)
            projection(0) = 0.0; // if X/Y/Z are ignored

        // TODO: restore original state, no?
    }

    virtual int luaProjectCallbackSize() const
    {
        return task->projectionEvaluation.dim;
    }

    virtual void luaProjectCallback(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        projectionEvaluationCallback_in in_args;
        projectionEvaluationCallback_out out_args;

        for(size_t i = 0; i < stateVec.size(); i++)
            in_args.state.push_back((float)stateVec[i]);

        if(projectionEvaluationCallback(task->projectionEvaluation.callback.scriptId, task->projectionEvaluation.callback.function.c_str(), &in_args, &out_args))
        {
            for(size_t i = 0; i < out_args.projection.size(); i++)
            {
                projection(i) = out_args.projection[i];
                std::cout << (i ? ", " : "") << out_args.projection[i];
            }
            std::cout << std::endl;
        }
        else
        {
            throw ompl::Exception("Projection evaluation callback " + task->projectionEvaluation.callback.function + " returned an error");
        }
    }

    TaskDef *task;
    const ob::StateSpacePtr& statespace;
    int dim;
};

class StateSpace : public ob::CompoundStateSpace
{
public:
    StateSpace(TaskDef *task)
        : ob::CompoundStateSpace(), task(task)
    {
        setName("VREPCompoundStateSpace");
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            ob::StateSpacePtr subSpace;

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                subSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                break;
            case sim_ompl_statespacetype_pose3d:
                subSpace = ob::StateSpacePtr(new ob::SE3StateSpace());
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                break;
            case sim_ompl_statespacetype_position2d:
                subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
                break;
            case sim_ompl_statespacetype_position3d:
                subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
                break;
            case sim_ompl_statespacetype_joint_position:
                subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
                break;
            case sim_ompl_statespacetype_dubins:
                subSpace = ob::StateSpacePtr(new ob::DubinsStateSpace(stateSpace->dubinsTurningRadius, stateSpace->dubinsIsSymmetric));
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                break;
            }

            subSpace->setName(stateSpace->header.name);
            addSubspace(subSpace, stateSpace->weight);

            // set bounds:

            ob::RealVectorBounds bounds(stateSpace->boundsLow.size());;
            for(size_t j = 0; j < stateSpace->boundsLow.size(); j++)
                bounds.setLow(j, stateSpace->boundsLow[j]);
            for(size_t j = 0; j < stateSpace->boundsHigh.size(); j++)
                bounds.setHigh(j, stateSpace->boundsHigh[j]);

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                as<ob::SE2StateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_pose3d:
                as<ob::SE3StateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_position2d:
                as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_position3d:
                as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_joint_position:
                as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_dubins:
                as<ob::SE2StateSpace>(i)->setBounds(bounds);
                break;
            }
        }
    }

    // writes state s to V-REP:
    void writeState(const ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        int j = 0;
        simFloat pos[3], orient[4], value;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                pos[0] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getY();
                orient[2] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                simSetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_pose3d:
                pos[0] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getY();
                pos[2] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                orient[0] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().x;
                orient[1] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().y;
                orient[2] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().z;
                orient[3] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().w;
                simSetObjectQuaternion(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_position2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                pos[0] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                pos[1] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_position3d:
                pos[0] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                pos[1] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                pos[2] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_joint_position:
                value = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                simSetJointPosition(stateSpace->objectHandle, value);
                break;
            case sim_ompl_statespacetype_dubins:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                pos[0] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getY();
                orient[2] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                simSetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            }
        }
    }

    // reads state s from V-REP:
    void readState(ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        simFloat pos[3], orient[4], value;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[2]);
                break;
            case sim_ompl_statespacetype_pose3d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectQuaternion(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                s->as<ob::SE3StateSpace::StateType>(i)->setXYZ(pos[0], pos[1], pos[2]);
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().x = orient[0];
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().y = orient[1];
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().z = orient[2];
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().w = orient[3];
                break;
            case sim_ompl_statespacetype_position2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                break;
            case sim_ompl_statespacetype_position3d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[2] = pos[2];
                break;
            case sim_ompl_statespacetype_joint_position:
                simGetJointPosition(stateSpace->objectHandle, &value);
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = value;
                break;
            case sim_ompl_statespacetype_dubins:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[2]);
                break;
            }
        }
    }

protected:
    TaskDef *task;
};

class StateValidityChecker : public ob::StateValidityChecker
{
public:
    StateValidityChecker(const ob::SpaceInformationPtr &si, TaskDef *task)
        : ob::StateValidityChecker(si), statespace(si->getStateSpace()), task(task)
    {
    }

    virtual ~StateValidityChecker()
    {
    }

    virtual bool isValid(const ob::State *state) const
    {
        switch(task->stateValidation.type)
        {
        case TaskDef::StateValidation::DEFAULT:
            return checkDefault(state);
        case TaskDef::StateValidation::CLLBACK:
            return checkCallback(state);
        }
        return false;
    }

protected:
    virtual bool checkDefault(const ob::State *state) const
    {
        //ob::CompoundStateSpace *ss = statespace->as<ob::CompoundStateSpace>();
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        // check collisions:
        bool inCollision = false;
        for(size_t i = 0; i < task->collisionPairHandles.size() / 2; i++)
        {
            if(task->collisionPairHandles[2 * i + 0] >= 0)
            {
                int r = simCheckCollision(task->collisionPairHandles[2 * i + 0], task->collisionPairHandles[2 * i + 1]);
                collision_count++;
                std::cout << "\nColiision Count is " <<collision_count<<std::endl;
                if(r > 0)
                {
                    inCollision = true;
                    break;
                }
            }
            if(inCollision) break;
        }

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);

        return !inCollision;
    }

    virtual bool checkCallback(const ob::State *state) const
    {
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        bool ret = false;

        stateValidationCallback_in in_args;
        stateValidationCallback_out out_args;

        for(size_t i = 0; i < stateVec.size(); i++)
            in_args.state.push_back((float)stateVec[i]);

        if(stateValidationCallback(task->stateValidation.callback.scriptId, task->stateValidation.callback.function.c_str(), &in_args, &out_args))
        {
            ret = out_args.valid;
        }
        else
        {
            throw ompl::Exception("State validation callback " + task->stateValidation.callback.function + " returned an error");
        }

        return ret;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
};

class Goal : public ob::Goal
{
public:
    Goal(const ob::SpaceInformationPtr &si, TaskDef *task, double tolerance = 1e-3)
        : ob::Goal(si), statespace(si->getStateSpace()), task(task), tolerance(tolerance)
    {
    }

    virtual bool isSatisfied(const ob::State *state) const
    {
        double distance = 0.0;
        return isSatisfied(state, &distance);
    }

    virtual bool isSatisfied(const ob::State *state, double *distance) const
    {
        switch(task->goal.type)
        {
        case TaskDef::Goal::STATE:
            // silence -Wswitch warning
            // if really type is STATE we are not using this class for goal check
            return false;
        case TaskDef::Goal::DUMMY_PAIR:
            return checkDummyPair(state, distance);
        case TaskDef::Goal::CLLBACK:
            return checkCallback(state, distance);
        }

        return false;
    }

protected:
    virtual bool checkDummyPair(const ob::State *state, double *distance) const
    {
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        if(task->goal.metric[3] == 0.0)
        { // ignore orientation
            float goalPos[3];
            float robotPos[3];
            simGetObjectPosition(task->goal.dummyPair.goalDummy, task->goal.refDummy, &goalPos[0]);
            simGetObjectPosition(task->goal.dummyPair.robotDummy, task->goal.refDummy, &robotPos[0]);
            *distance = sqrt(pow((goalPos[0] - robotPos[0])*task->goal.metric[0], 2) + pow((goalPos[1] - robotPos[1])*task->goal.metric[1], 2) + pow((goalPos[2] - robotPos[2])*task->goal.metric[2], 2));
        }
        else
        { // do not ignore orientation
            float goalM[12];
            float robotM[12];
            simGetObjectMatrix(task->goal.dummyPair.goalDummy, task->goal.refDummy, goalM);
            simGetObjectMatrix(task->goal.dummyPair.robotDummy, task->goal.refDummy, robotM);
            float axis[3];
            float angle;
            simGetRotationAxis(robotM, goalM, axis, &angle);
            *distance = sqrt(pow((goalM[3] - robotM[3])*task->goal.metric[0], 2) + pow((goalM[7] - robotM[7])*task->goal.metric[1], 2) + pow((goalM[11] - robotM[11])*task->goal.metric[2], 2) + pow(angle*task->goal.metric[3], 2));
        }

        bool satisfied = *distance <= tolerance;

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);

        return satisfied;
    }

    virtual bool checkCallback(const ob::State *state, double *distance) const
    {
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        bool ret = false;

        goalCallback_in in_args;
        goalCallback_out out_args;

        for(size_t i = 0; i < stateVec.size(); i++)
            in_args.state.push_back((float)stateVec[i]);

        if(goalCallback(task->goal.callback.scriptId, task->goal.callback.function.c_str(), &in_args, &out_args))
        {
            ret = out_args.satisfied;
            *distance = out_args.distance;
        }
        else
        {
            throw ompl::Exception("Goal callback " + task->goal.callback.function + " returned an error");
        }

        return ret;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
    double tolerance;
};

class ValidStateSampler : public ob::UniformValidStateSampler
{
public:
    ValidStateSampler(const ob::SpaceInformation *si, TaskDef *task)
        : ob::UniformValidStateSampler(si), task(task)
    {
        name_ = "VREPValidStateSampler";
    }

    bool sample(ob::State *state)
    {
        if(task->validStateSampling.type == TaskDef::ValidStateSampling::CLLBACK)
        {
            if(task->validStateSampling.callback.function == "")
            {
                throw ompl::Exception("Specified empty callback for valid state sampling");
            }

            bool ret = false;

            validStateSamplerCallback_in in_args;
            validStateSamplerCallback_out out_args;

            if(validStateSamplerCallback(task->validStateSampling.callback.scriptId, task->validStateSampling.callback.function.c_str(), &in_args, &out_args))
            {
                std::vector<double> stateVec;
                for(size_t i = 0; i < out_args.sampledState.size(); i++)
                    stateVec.push_back((double)out_args.sampledState[i]);
                task->stateSpacePtr->copyFromReals(state, stateVec);
                ret = true;
            }
            else
            {
                throw ompl::Exception("Valid state sampling callback " + task->validStateSampling.callback.function + " returned an error");
            }

            return ret;
        }
        else
        {
            return ob::UniformValidStateSampler::sample(state);
        }
    }

    bool sampleNear(ob::State *state, const ob::State *nearState, const double distance)
    {
        if(task->validStateSampling.type == TaskDef::ValidStateSampling::CLLBACK)
        {
            if(task->validStateSampling.callbackNear.function == "")
            {
                throw ompl::Exception("Specified empty callback for \"near\" valid state sampling");
            }

            std::vector<double> nearStateVec;
            task->stateSpacePtr->copyToReals(nearStateVec, nearState);

            bool ret = false;

            validStateSamplerCallbackNear_in in_args;
            validStateSamplerCallbackNear_out out_args;

            for(size_t i = 0; i < nearStateVec.size(); i++)
                in_args.state.push_back((float)nearStateVec[i]);
            in_args.distance = distance;

            if(validStateSamplerCallbackNear(task->validStateSampling.callbackNear.scriptId, task->validStateSampling.callbackNear.function.c_str(), &in_args, &out_args))
            {
                std::vector<double> stateVec;
                for(size_t i = 0; i < out_args.sampledState.size(); i++)
                    stateVec.push_back((double)out_args.sampledState[i]);
                task->stateSpacePtr->copyFromReals(state, stateVec);
                ret = true;
            }
            else
            {
                throw ompl::Exception("Near valid state sampling callback " + task->validStateSampling.callbackNear.function + " returned an error");
            }

            return ret;
        }
        else
        {
            return ob::UniformValidStateSampler::sampleNear(state, nearState, distance);
        }
    }

protected:
    TaskDef *task;
};

typedef std::shared_ptr<ValidStateSampler> ValidStateSamplerPtr;

ob::ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si, TaskDef *task)
{
    return ob::ValidStateSamplerPtr(new ValidStateSampler(si, task));
}

void createStateSpace(SScriptCallBack *p, const char *cmd, createStateSpace_in *in, createStateSpace_out *out)
{
    if(in->boundsLow.size() != in->boundsHigh.size())
        throw std::string("Lower and upper bounds must have the same length.");

    if(in->weight <= 0)
        throw std::string("State component weight must be positive.");

    if(in->refObjectHandle != -1 && simIsHandleValid(in->refObjectHandle, sim_appobj_object_type) <= 0)
        throw std::string("Reference object handle is not valid.");

    StateSpaceDef *statespace = new StateSpaceDef();
    statespace->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
    statespace->header.handle = nextStateSpaceHandle++;
    statespace->header.name = in->name;
    statespace->type = static_cast<StateSpaceType>(in->type);
    statespace->objectHandle = in->objectHandle;
    for(size_t i = 0; i < in->boundsLow.size(); i++)
        statespace->boundsLow.push_back(in->boundsLow[i]);
    for(size_t i = 0; i < in->boundsHigh.size(); i++)
        statespace->boundsHigh.push_back(in->boundsHigh[i]);
    statespace->defaultProjection = in->useForProjection > 0;
    statespace->weight = in->weight;
    statespace->refFrameHandle = in->refObjectHandle;
    statespace->dubinsTurningRadius = 0.05;
    statespace->dubinsIsSymmetric = false;
    statespaces[statespace->header.handle] = statespace;
    out->stateSpaceHandle = statespace->header.handle;
}

void destroyStateSpace(SScriptCallBack *p, const char *cmd, destroyStateSpace_in *in, destroyStateSpace_out *out)
{
    if(statespaces.find(in->stateSpaceHandle) == statespaces.end())
        throw std::string("Invalid state space handle.");

    StateSpaceDef *statespace = statespaces[in->stateSpaceHandle];
    statespaces.erase(in->stateSpaceHandle);
    delete statespace;
}

void setDubinsParams(SScriptCallBack *p, const char *cmd, setDubinsParams_in *in, setDubinsParams_out *out)
{
    if(statespaces.find(in->stateSpaceHandle) == statespaces.end())
        throw std::string("Invalid state space handle.");

    StateSpaceDef *statespace = statespaces[in->stateSpaceHandle];
    statespace->dubinsTurningRadius = in->turningRadius;
    statespace->dubinsIsSymmetric = in->isSymmetric;
}

void createTask(SScriptCallBack *p, const char *cmd, createTask_in *in, createTask_out *out)
{
    TaskDef *task = new TaskDef();
    task->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
    task->header.handle = nextTaskHandle++;
    task->header.name = in->name;
    task->goal.type = TaskDef::Goal::STATE;
    task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
    task->stateValidityCheckingResolution = 0.01f; // 1% of state space's extent
    task->validStateSampling.type = TaskDef::ValidStateSampling::DEFAULT;
    task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
    task->algorithm = sim_ompl_algorithm_KPIECE1;
    task->verboseLevel = 0;
    tasks[task->header.handle] = task;
    out->taskHandle = task->header.handle;
}

TaskDef * getTask(simInt taskHandle)
{
    if(tasks.find(taskHandle) == tasks.end())
        throw std::string("Invalid task handle.");

    return tasks[taskHandle];
}

void destroyTask(SScriptCallBack *p, const char *cmd, destroyTask_in *in, destroyTask_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    tasks.erase(in->taskHandle);
    delete task;
}

void printTaskInfo(SScriptCallBack *p, const char *cmd, printTaskInfo_in *in, printTaskInfo_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    std::stringstream s;
    std::string prefix = "OMPL: ";
    s << prefix << "task name: " << task->header.name << std::endl;
    s << prefix << "state spaces: (dimension: " << task->dim << ")" << std::endl;
    for(size_t i = 0; i < task->stateSpaces.size(); i++)
    {
        StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];
        s << prefix << "    state space: " << stateSpace->header.handle << std::endl;
        s << prefix << "        name: " << stateSpace->header.name << std::endl;
        s << prefix << "        type: " << statespacetype_string(stateSpace->type) << std::endl;
        s << prefix << "        object handle: " << stateSpace->objectHandle << std::endl;
        s << prefix << "        bounds low: {";
        for(size_t j = 0; j < stateSpace->boundsLow.size(); j++)
            s << (j ? ", " : "") << stateSpace->boundsLow[j];
        s << "}" << std::endl;
        s << prefix << "        bounds high: {";
        for(size_t j = 0; j < stateSpace->boundsHigh.size(); j++)
            s << (j ? ", " : "") << stateSpace->boundsHigh[j];
        s << "}" << std::endl;
        s << prefix << "        default projection: " << (stateSpace->defaultProjection ? "true" : "false") << std::endl;
        s << prefix << "        weight: " << stateSpace->weight << std::endl;
        if(stateSpace->type == sim_ompl_statespacetype_dubins)
        {
            s << prefix << "        (dubins) turning radius: " << stateSpace->dubinsTurningRadius << std::endl;
            s << prefix << "        (dubins) symmetric: " << (stateSpace->dubinsIsSymmetric ? "true" : "false") << std::endl;
        }
    }
    s << prefix << "collision pairs: {";
    for(size_t i = 0; i < task->collisionPairHandles.size(); i++)
        s << (i ? ", " : "") << task->collisionPairHandles[i];
    s << "}" << std::endl;
    s << prefix << "start state: {";
    for(size_t i = 0; i < task->startState.size(); i++)
        s << (i ? ", " : "") << task->startState[i];
    s << "}" << std::endl;
    s << prefix << "goal:";
    switch(task->goal.type)
    {
    case TaskDef::Goal::STATE:
        s << std::endl;
        s << prefix << "    goal state(s):" << std::endl;
        for(size_t j = 0; j < task->goal.states.size(); j++)
        {
            s << prefix << "        {";
            for(size_t i = 0; i < task->goal.states[j].size(); i++)
                s << (i ? ", " : "") << task->goal.states[j][i];
            s << "}" << std::endl;
        }
        break;
    case TaskDef::Goal::DUMMY_PAIR:
        s << std::endl;
        s << prefix << "    robot dummy: " << task->goal.dummyPair.robotDummy << std::endl;
        s << prefix << "    goal dummy: " << task->goal.dummyPair.goalDummy << std::endl;
        s << prefix << "    ref dummy: " << task->goal.refDummy << std::endl;
        s << prefix << "    metric: {";
        for(size_t i = 0; i < 4; i++)
            s << (i ? ", " : "") << task->goal.metric[i];
        s << "}" << std::endl;
        s << prefix << "    tolerance: " << task->goal.tolerance << std::endl;
        break;
    case TaskDef::Goal::CLLBACK:
        s << std::endl;
        s << prefix << "    callback:" << std::endl;
        s << prefix << "        scriptId: " << task->goal.callback.scriptId << std::endl;
        s << prefix << "        function: " << task->goal.callback.function << std::endl;
        break;
    default:
        s << " ???" << std::endl;
        break;
    }
    s << prefix << "state validation:";
    switch(task->stateValidation.type)
    {
    case TaskDef::StateValidation::DEFAULT:
        s << " default" << std::endl;
        break;
    case TaskDef::StateValidation::CLLBACK:
        s << std::endl;
        s << prefix << "    callback:" << std::endl;
        s << prefix << "        scriptId: " << task->stateValidation.callback.scriptId << std::endl;
        s << prefix << "        function: " << task->stateValidation.callback.function << std::endl;
        break;
    default:
        s << " ???" << std::endl;
        break;
    }
    s << prefix << "state validity checking resolution: " << task->stateValidityCheckingResolution << std::endl;
    s << prefix << "valid state sampling:";
    switch(task->validStateSampling.type)
    {
    case TaskDef::ValidStateSampling::DEFAULT:
        s << " default" << std::endl;
        break;
    case TaskDef::ValidStateSampling::CLLBACK:
        s << std::endl;
        s << prefix << "    callback:" << std::endl;
        s << prefix << "        scriptId: " << task->validStateSampling.callback.scriptId << std::endl;
        s << prefix << "        function: " << task->validStateSampling.callback.function << std::endl;
        s << prefix << "    callbackNear:" << std::endl;
        s << prefix << "        scriptId: " << task->validStateSampling.callbackNear.scriptId << std::endl;
        s << prefix << "        function: " << task->validStateSampling.callbackNear.function << std::endl;
        break;
    }
    s << prefix << "projection evaluation:";
    switch(task->projectionEvaluation.type)
    {
    case TaskDef::ProjectionEvaluation::DEFAULT:
        s << " default" << std::endl;
        break;
    case TaskDef::ProjectionEvaluation::CLLBACK:
        s << std::endl;
        s << prefix << "    callback:" << std::endl;
        s << prefix << "        scriptId:" << task->projectionEvaluation.callback.scriptId << std::endl;
        s << prefix << "        function:" << task->projectionEvaluation.callback.function << std::endl;
        break;
    default:
        s << " ???" << std::endl;
        break;
    }
    s << prefix << "algorithm: " << algorithm_string(task->algorithm) << std::endl;

    simAddStatusbarMessage(s.str().c_str());
    std::cout << s.str();
}

void setVerboseLevel(SScriptCallBack *p, const char *cmd, setVerboseLevel_in *in, setVerboseLevel_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    task->verboseLevel = in->verboseLevel;
}

void setStateValidityCheckingResolution(SScriptCallBack *p, const char *cmd, setStateValidityCheckingResolution_in *in, setStateValidityCheckingResolution_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    task->stateValidityCheckingResolution = in->resolution;
}

void setStateSpace(SScriptCallBack *p, const char *cmd, setStateSpace_in *in, setStateSpace_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    bool valid_statespace_handles = true;

    for(size_t i = 0; i < in->stateSpaceHandles.size(); i++)
    {
        simInt stateSpaceHandle = in->stateSpaceHandles[i];

        if(statespaces.find(stateSpaceHandle) == statespaces.end())
        {
            valid_statespace_handles = false;
            break;
        }
    }

    if(!valid_statespace_handles)
        throw std::string("Invalid state space handle.");

    task->stateSpaces.clear();
    task->dim = 0;
    for(size_t i = 0; i < in->stateSpaceHandles.size(); i++)
    {
        simInt stateSpaceHandle = in->stateSpaceHandles[i];
        task->stateSpaces.push_back(stateSpaceHandle);
        switch(statespaces.find(stateSpaceHandle)->second->type)
        {
        case sim_ompl_statespacetype_position2d:
            task->dim += 2;
            break;
        case sim_ompl_statespacetype_pose2d:
            task->dim += 3;
            break;
        case sim_ompl_statespacetype_position3d:
            task->dim += 3;
            break;
        case sim_ompl_statespacetype_pose3d:
            task->dim += 7;
            break;
        case sim_ompl_statespacetype_joint_position:
            task->dim += 1;
            break;
        case sim_ompl_statespacetype_dubins:
            task->dim += 3;
            break;
        }
    }
}

void setAlgorithm(SScriptCallBack *p, const char *cmd, setAlgorithm_in *in, setAlgorithm_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    task->algorithm = static_cast<Algorithm>(in->algorithm);
}

void setCollisionPairs(SScriptCallBack *p, const char *cmd, setCollisionPairs_in *in, setCollisionPairs_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    int numHandles = (in->collisionPairHandles.size() / 2) * 2;
    task->collisionPairHandles.clear();
    for(int i = 0; i < numHandles; i++)
        task->collisionPairHandles.push_back(in->collisionPairHandles[i]);
}

void validateStateSize(const TaskDef *task, const std::vector<float>& s, std::string descr = "State")
{
    if(s.size() == 0)
        throw descr + " is empty.";
    if(s.size() != task->dim)
    {
        std::stringstream ss;
        ss << descr << " is of incorrect size. Expected " << task->dim << ", got " << s.size() << ".";
        if(task->dim == 0)
            ss << " Did you forget to set the state space for this task?";
        throw ss.str();
    }
}

void setStartState(SScriptCallBack *p, const char *cmd, setStartState_in *in, setStartState_out *out)
{
    TaskDef *task = getTask(in->taskHandle);
    validateStateSize(task, in->state);

    task->startState.clear();
    for(size_t i = 0; i < in->state.size(); i++)
        task->startState.push_back(in->state[i]);

    // for multi-query PRM, if the OMPL's ProblemDefinition has already been set,
    // we want only to clear the query and add the new start state:
    if(task->problemDefinitionPtr && task->planner && task->algorithm == sim_ompl_algorithm_PRM)
    {
        task->planner->as<og::PRM>()->clearQuery();
        ob::ScopedState<> startState(task->stateSpacePtr);
        for(size_t i = 0; i < task->startState.size(); i++)
            startState[i] = task->startState[i];
        task->problemDefinitionPtr->clearStartStates();
        task->problemDefinitionPtr->addStartState(startState);
    }
}

void setGoalState(SScriptCallBack *p, const char *cmd, setGoalState_in *in, setGoalState_out *out)
{
    TaskDef *task = getTask(in->taskHandle);
    validateStateSize(task, in->state);

    task->goal.type = TaskDef::Goal::STATE;
    task->goal.states.clear();
    task->goal.states.push_back(std::vector<simFloat>());

    for(size_t i = 0; i < in->state.size(); i++)
        task->goal.states[0].push_back(in->state[i]);
}

void addGoalState(SScriptCallBack *p, const char *cmd, addGoalState_in *in, addGoalState_out *out)
{
    TaskDef *task = getTask(in->taskHandle);
    validateStateSize(task, in->state);

    task->goal.type = TaskDef::Goal::STATE;

    size_t last = task->goal.states.size();
    task->goal.states.push_back(std::vector<simFloat>());

    for(size_t i = 0; i < in->state.size(); i++)
        task->goal.states[last].push_back(in->state[i]);
}

void setGoal(SScriptCallBack *p, const char *cmd, setGoal_in *in, setGoal_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    task->goal.type = TaskDef::Goal::DUMMY_PAIR;
    task->goal.dummyPair.goalDummy = in->goalDummy;
    task->goal.dummyPair.robotDummy = in->robotDummy;

    task->goal.tolerance = in->tolerance;

    task->goal.metric[0] = in->metric[0];
    task->goal.metric[1] = in->metric[1];
    task->goal.metric[2] = in->metric[2];
    task->goal.metric[3] = in->metric[3];

    task->goal.refDummy = in->refDummy;
}

ob::PlannerPtr plannerFactory(Algorithm algorithm, ob::SpaceInformationPtr si)
{
    ob::PlannerPtr planner;
#define PLANNER(x) case sim_ompl_algorithm_##x: planner = ob::PlannerPtr(new og::x(si)); break
    switch(algorithm)
    {
        PLANNER(BiTRRT);
        PLANNER(BITstar);
        PLANNER(BKPIECE1);
        PLANNER(CForest);
        PLANNER(EST); // needs projection
        PLANNER(FMT);
        PLANNER(KPIECE1); // needs projection
        PLANNER(LazyPRM);
        PLANNER(LazyPRMstar);
        PLANNER(LazyRRT);
        PLANNER(LBKPIECE1);
        PLANNER(LBTRRT);
        //PLANNER(LightningRetrieveRepair);
        PLANNER(PDST); // needs projection
        PLANNER(PRM);
        PLANNER(PRMstar);
        PLANNER(pRRT);
        PLANNER(pSBL);
        PLANNER(RRT);
        PLANNER(RRTConnect);
        PLANNER(RRTstar);
        PLANNER(SBL); // needs projection
        PLANNER(SPARS);
        PLANNER(SPARStwo);
        PLANNER(STRIDE);
        PLANNER(TRRT);
    }
#undef PLANNER
    return planner;
}

void setup(SScriptCallBack *p, const char *cmd, setup_in *in, setup_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    task->stateSpacePtr = ob::StateSpacePtr(new StateSpace(task));
    task->spaceInformationPtr = ob::SpaceInformationPtr(new ob::SpaceInformation(task->stateSpacePtr));
    task->projectionEvaluatorPtr = ob::ProjectionEvaluatorPtr(new ProjectionEvaluator(task->stateSpacePtr, task));
    task->stateSpacePtr->registerDefaultProjection(task->projectionEvaluatorPtr);
    task->problemDefinitionPtr = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(task->spaceInformationPtr));
    task->spaceInformationPtr->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(task->spaceInformationPtr, task)));
    task->spaceInformationPtr->setStateValidityCheckingResolution(task->stateValidityCheckingResolution);
    task->spaceInformationPtr->setValidStateSamplerAllocator(std::bind(allocValidStateSampler, std::placeholders::_1, task));

    ob::ScopedState<> startState(task->stateSpacePtr);
    validateStateSize(task, task->startState, "Start state");
    for(size_t i = 0; i < task->startState.size(); i++)
        startState[i] = task->startState[i];
    task->problemDefinitionPtr->addStartState(startState);

    ob::GoalPtr goal;
    if(task->goal.type == TaskDef::Goal::STATE)
    {
        for(size_t i = 0; i < task->goal.states.size(); i++)
            validateStateSize(task, task->goal.states[i], "Goal state");

        if(task->goal.states.size() > 1)
        {
            goal = ob::GoalPtr(new ob::GoalStates(task->spaceInformationPtr));
            for(size_t j = 0; j < task->goal.states.size(); j++)
            {
                ob::ScopedState<> goalState(task->stateSpacePtr);
                for(size_t i = 0; i < task->goal.states[j].size(); i++)
                    goalState[i] = task->goal.states[j][i];
                goal->as<ob::GoalStates>()->addState(goalState);
            }
        }
        else if(task->goal.states.size() == 1)
        {
            goal = ob::GoalPtr(new ob::GoalState(task->spaceInformationPtr));
            ob::ScopedState<> goalState(task->stateSpacePtr);
            for(size_t i = 0; i < task->goal.states[0].size(); i++)
                goalState[i] = task->goal.states[0][i];
            goal->as<ob::GoalState>()->setState(goalState);
        }
        else
        {
            throw std::string("No goal state specified.");
        }
    }
    else if(task->goal.type == TaskDef::Goal::DUMMY_PAIR || task->goal.type == TaskDef::Goal::CLLBACK)
    {
        goal = ob::GoalPtr(new Goal(task->spaceInformationPtr, task, (double)task->goal.tolerance));
    }
    task->problemDefinitionPtr->setGoal(goal);

    task->planner = plannerFactory(task->algorithm, task->spaceInformationPtr);
    if(!task->planner)
    {
        throw std::string("Invalid motion planning algorithm.");
    }
    task->planner->setProblemDefinition(task->problemDefinitionPtr);
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
   int i;
   for(i = 0; i<argc; i++) {
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}

void solve(SScriptCallBack *p, const char *cmd, solve_in *in, solve_out *out)
{

    std::string sql;
    char *zErrMsg=0;
    int rc;
    sqlite3* db = nullptr;
    sqlite3_open("/home/lamy/Desktop/OMPL_Compare_Task/VREP_Test_Maps/scenarios.db", &db);
    sql = "INSERT INTO Lamy_results (Collision_Count) VALUES (" + std::to_string(collision_count) + ")";
    char* sqlstate = strdup(sql.c_str());
    rc = sqlite3_exec(db,sqlstate,callback,0,&zErrMsg);
    sqlite3_close(db);

    std::cout << "\nColiision Count is " <<collision_count<<std::endl;
    //collision_count=0;
    TaskDef *task = getTask(in->taskHandle);
    ob::PlannerStatus solved = task->planner->solve(in->maxTime);
    if(solved)
    {
        out->solved = true;

        if(task->verboseLevel >= 1)
        {
            const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
            og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

            simAddStatusbarMessage("OMPL: found solution:");
            std::stringstream s;
            path.print(s);
            simAddStatusbarMessage(s.str().c_str());
        }
    }
    else
    {
        out->solved = false;

        if(task->verboseLevel >= 1)
            simAddStatusbarMessage("OMPL: could not find solution.");
    }
}

void simplifyPath(SScriptCallBack *p, const char *cmd, simplifyPath_in *in, simplifyPath_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(task->verboseLevel >= 2)
        simAddStatusbarMessage("OMPL: simplifying solution...");

    const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
    og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);
    og::PathSimplifierPtr pathSimplifier(new og::PathSimplifier(task->spaceInformationPtr));
    if(in->maxSimplificationTime < -std::numeric_limits<double>::epsilon())
        pathSimplifier->simplifyMax(path);
    else
        pathSimplifier->simplify(path, in->maxSimplificationTime);

    if(task->verboseLevel >= 1)
    {
        simAddStatusbarMessage("OMPL: simplified solution:");
        std::stringstream s;
        path.print(s);
        simAddStatusbarMessage(s.str().c_str());
    }
}

void interpolatePath(SScriptCallBack *p, const char *cmd, interpolatePath_in *in, interpolatePath_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(task->verboseLevel >= 2)
        simAddStatusbarMessage("OMPL: interpolating solution...");

    const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
    og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

    if(in->stateCnt == 0)
        path.interpolate(); // this doesn't give the same result as path.interpolate(0) as I thought!!
    else
        path.interpolate(in->stateCnt);

    if(task->verboseLevel >= 2)
    {
        simAddStatusbarMessage("OMPL: interpolated:");
        std::stringstream s;
        path.print(s);
        simAddStatusbarMessage(s.str().c_str());
    }
}

void getPath(SScriptCallBack *p, const char *cmd, getPath_in *in, getPath_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
    og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

    for(size_t i = 0; i < path.getStateCount(); i++)
    {
        const ob::StateSpace::StateType *s = path.getState(i);
        std::vector<double> v;
        task->stateSpacePtr->copyToReals(v, s);
        for(size_t j = 0; j < v.size(); j++)
            out->states.push_back((float)v[j]);
    }
}

void getData(SScriptCallBack *p, const char *cmd, getData_in *in, getData_out *out)
{
    TaskDef *task = getTask(in->taskHandle);
    float min_value = 1000.0f, max_value = 0.0f;

    std::vector<unsigned int> edge_list;
    ompl::base::PlannerData data(task->spaceInformationPtr);
    task->planner->getPlannerData(data);

    for(unsigned int i = 0; i < data.numVertices(); i++)
    {
        ompl::base::PlannerDataVertex v(data.getVertex(i));
        int tag = v.getTag(); // Minimum distance to the closest X_obs for AdaptiveLazyPRM*.
        float radius;
        memcpy(&radius, &tag, sizeof(float));

        const ob::StateSpace::StateType *state = v.getState();
        std::vector<double> config;
        task->stateSpacePtr->copyToReals(config, state);
        for(unsigned int j = 0; j < config.size(); j++)
            out->states.push_back((float)config[j]);

        config.clear();
        out->states.push_back(radius);
    }
}

void compute(SScriptCallBack *p, const char *cmd, compute_in *in, compute_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    setup(p, in->taskHandle);
    out->solved = solve(p, in->taskHandle, in->maxTime);
    if(!out->solved) return;
    simplifyPath(p, in->taskHandle, in->maxSimplificationTime);
    interpolatePath(p, in->taskHandle, in->stateCnt);
    out->states = getPath(p, in->taskHandle);
}

void readState(SScriptCallBack *p, const char *cmd, readState_in *in, readState_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(!task->stateSpacePtr)
        throw std::string("This method can only be used inside callbacks.");

    ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
    task->stateSpacePtr->as<StateSpace>()->readState(state);
    std::vector<double> stateVec = state.reals();
    for(size_t i = 0; i < stateVec.size(); i++)
        out->state.push_back((float)stateVec[i]);
}

void writeState(SScriptCallBack *p, const char *cmd, writeState_in *in, writeState_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(!task->stateSpacePtr)
        throw std::string("This method can only be used inside callbacks.");

    validateStateSize(task, in->state);

    ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
    for(int i = 0; i < task->dim; i++)
        state[i] = (double)in->state[i];
    task->stateSpacePtr->as<StateSpace>()->writeState(state);
}

void isStateValid(SScriptCallBack *p, const char *cmd, isStateValid_in *in, isStateValid_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(!task->stateSpacePtr)
        throw std::string("This method can only be used inside callbacks.");

    validateStateSize(task, in->state);

    std::vector<double> stateVec;
    for(size_t i = 0; i < in->state.size(); i++)
        stateVec.push_back((double)in->state[i]);
    ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
    ob::State *s = &(*state);
    task->stateSpacePtr->copyFromReals(s, stateVec);

    out->valid = task->spaceInformationPtr->isValid(s) ? 1 : 0;
}

void setProjectionEvaluationCallback(SScriptCallBack *p, const char *cmd, setProjectionEvaluationCallback_in *in, setProjectionEvaluationCallback_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(in->projectionSize < 1)
        throw std::string("Projection size must be positive.");

    if(in->callback == "")
    {
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
        task->projectionEvaluation.dim = 0;
        task->projectionEvaluation.callback.scriptId = 0;
        task->projectionEvaluation.callback.function = "";
    }
    else
    {
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::CLLBACK;
        task->projectionEvaluation.dim = in->projectionSize;
        task->projectionEvaluation.callback.scriptId = p->scriptID;
        task->projectionEvaluation.callback.function = in->callback;
    }
}

void setStateValidationCallback(SScriptCallBack *p, const char *cmd, setStateValidationCallback_in *in, setStateValidationCallback_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(in->callback == "")
    {
        task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
        task->stateValidation.callback.scriptId = 0;
        task->stateValidation.callback.function = "";
    }
    else
    {
        task->stateValidation.type = TaskDef::StateValidation::CLLBACK;
        task->stateValidation.callback.scriptId = p->scriptID;
        task->stateValidation.callback.function = in->callback;
    }
}

void setGoalCallback(SScriptCallBack *p, const char *cmd, setGoalCallback_in *in, setGoalCallback_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(in->callback == "")
        throw std::string("Invalid callback name.");
 
    task->goal.type = TaskDef::Goal::CLLBACK;
    task->goal.callback.scriptId = p->scriptID;
    task->goal.callback.function = in->callback;
}

void setValidStateSamplerCallback(SScriptCallBack *p, const char *cmd, setValidStateSamplerCallback_in *in, setValidStateSamplerCallback_out *out)
{
    TaskDef *task = getTask(in->taskHandle);

    if(in->callback == "" || in->callbackNear == "")
        throw std::string("Invalid callback name.");

    task->validStateSampling.type = TaskDef::ValidStateSampling::CLLBACK;
    task->validStateSampling.callback.scriptId = p->scriptID;
    task->validStateSampling.callback.function = in->callback;
    task->validStateSampling.callbackNear.scriptId = p->scriptID;
    task->validStateSampling.callbackNear.function = in->callbackNear;
}

class Plugin : public vrep::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        simSetModuleInfo(PLUGIN_NAME, 0, "OMPL (open motion planning library) Plugin", 0);
        simSetModuleInfo(PLUGIN_NAME, 1, BUILD_DATE, 0);
    }

    void onSimulationEnded()
    {
        destroyTransientObjects();
    }
};

VREP_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
