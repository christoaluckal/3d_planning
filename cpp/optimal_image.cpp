/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/SpaceInformation.h>

#include <ompl/util/PPM.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <random>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ValidityChecker : public ob::StateValidityChecker
 {
 public:
    
     ValidityChecker(const ob::SpaceInformationPtr& si) :
         ob::StateValidityChecker(si) {

         }
  
     // Returns whether the given state's position overlaps the
     // circular obstacle
     bool isValid(const ob::State* state) const override
     {
        const auto* state2D = state->as<ob::RealVectorStateSpace::StateType>();
        int row_val = state2D->values[0];
        int col_val = state2D->values[1];

        if(row_val < 0 || row_val > 100 || col_val < 0 || col_val > 100)
        {
            return false;
        }

        return true;
     }

 };

class CustomClearance : public ob::OptimizationObjective
{
public:
CustomClearance(const ob::SpaceInformationPtr &si) :
    ob::OptimizationObjective(si) {}

    virtual ob::Cost stateCost(const ob::State* s) const;
    virtual bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const;
    virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
    virtual ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const;
    virtual ob::Cost identityCost() const;
    virtual ob::Cost infiniteCost() const;
};

ob::Cost CustomClearance::stateCost(const ob::State* s) const
{
// std::cout << "stateCost\n";
    const auto* state2D =
                s->as<ob::RealVectorStateSpace::StateType>();
    // return ob::Cost(this->si_->getStateValidityChecker()->clearance(s));
    // if(state2D->values[0]<10 && state2D->values[1]<10)
    // { 
    //     return ob::Cost(double(5));
    // }
    // else{
    //     return ob::Cost(double(1));
    // }

    double rand = std::rand() % 10;

    return ob::Cost(double(rand));

}

bool CustomClearance::isCostBetterThan(ob::Cost c1, ob::Cost c2) const
{
// std::cout << "isCostBetterThan\n";
return c1.value() < c2.value();
}

ob::Cost CustomClearance::combineCosts(ob::Cost c1, ob::Cost c2) const
{
// std::cout << "combineCosts\n";
// if (c1.value() < c2.value())
//     return c1;
// else
//     return c2;

return ob::Cost(c1.value()+c2.value());
}

ob::Cost CustomClearance::identityCost() const
{
// std::cout << "identityCost\n";
// return ob::Cost(std::numeric_limits<double>::infinity());
return ob::Cost(0);
}

ob::Cost CustomClearance::infiniteCost() const
{
// std::cout << "infiniteCost\n";
return ob::Cost(std::numeric_limits<double>::infinity());
}

ob::Cost CustomClearance::motionCost(const ob::State *s1, const ob::State *s2) const
{
// std::cout << "motionCost\n";
return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
}

ob::OptimizationObjectivePtr CustomObjective(const ob::SpaceInformationPtr& si)
{
// ob::OptimizationObjectivePtr obj(new CustomClearance(si));
// return obj;

return std::make_shared<CustomClearance>(si);
}


class Plane2DEnvironment
{
public:
    Plane2DEnvironment(const char *ppm_file, bool use_deterministic_sampling = false)
    {
        bool ok = false;
        useDeterministicSampling_ = use_deterministic_sampling;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch (ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        auto space(std::make_shared<ob::RealVectorStateSpace>());
        space->addDimension(0.0, ppm_.getWidth());
        space->addDimension(0.0, ppm_.getHeight());
        auto si(std::make_shared<ob::SpaceInformation>(space));
        maxWidth_ = ppm_.getWidth() - 1;
        maxHeight_ = ppm_.getHeight() - 1;
        si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));
        si->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
        si->setup();

        ob::ScopedState<> start(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_row;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_col;
    
        // Set our robot's goal state to be the top-right corner of the
        // environment, or (1,1).
        ob::ScopedState<> goal(space);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_row;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_col;

  
        // Create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    
        // Set the start and goal states
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(CustomObjective(si));

        ob::PlannerPtr p = std::make_shared<og::RRTstar>(si);

        p->setProblemDefinition(pdef);
        p->setup();
    
        // attempt to solve the planning problem in the given runtime
        ob::PlannerStatus solved = p->solve(1);
    
        if (solved)
        {
            // Output the length of the path found
            std::cout
            << p->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

            std::vector<ob::State*> fstates;
            pdef->getSolutionPath()->print(std::cout);
            
            // take cout path and put it in a string
            std::stringstream ss;
            pdef->getSolutionPath()->print(ss);
            
            // dump the path to a file
            std::ofstream myfile;
            myfile.open("path.txt");
            myfile << ss.str();
            myfile.close();

            return true;
        }
        else
            std::cout << "No solution found." << std::endl;

            return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h =
                std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
        std::cout << "Saved the result to '" << filename << "'" << std::endl;
    }

private:
    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        double gray = 0.2126 * c.red + 0.7152 * c.green + 0.0722 * c.blue;
    //  return c.red > 127 && c.green > 127 && c.blue > 127;
    return gray < 127;
    }

    ob::StateSamplerPtr allocateHaltonStateSamplerRealVector(const ompl::base::StateSpace *space, unsigned int dim,
                                                            std::vector<unsigned int> bases = {})
    {
        // specify which deterministic sequence to use, here: HaltonSequence
        // optionally we can specify the bases used for generation (otherwise first dim prime numbers are used)
        if (bases.size() != 0)
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(bases.size(), bases));
        else
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(dim));
    }

    og::SimpleSetupPtr ss_;
    ob::SpaceInformationPtr si_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    bool useDeterministicSampling_;
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    bool useDeterministicSampling = false;
    std::string env_file = "/home/caluckal/Developer/summer2024/3d_planning/py_scripts/landscape.ppm";
//  Plane2DEnvironment env("/home/christoa/Developer/summer2024/3d_planning/py_scripts/landscape.ppm".c_str(), useDeterministicSampling);
Plane2DEnvironment env(env_file.c_str(), useDeterministicSampling);
    
    if (env.plan(0, 0, 80, 80))
    {
        std::cout << "Plan successful\n";
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}