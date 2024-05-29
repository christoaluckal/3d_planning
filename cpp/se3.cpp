/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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
  
 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/SE3StateSpace.h>
 #include <ompl/geometric/planners/prm/PRM.h>
 #include <ompl/geometric/SimpleSetup.h>
  
 #include <ompl/config.h>
 #include <iostream>
 #include <vector>
 #include <boost/format.hpp>
 #include <fstream>

  
 namespace ob = ompl::base;
 namespace og = ompl::geometric;

 class Circle
 {
    public:
        float x;
        float y;
        float r;
        Circle(float x, float y, float r)
        {
            this->x = x;
            this->y = y;
            this->r = r;
        }

        bool isInside(float x, float y)
        {
            return (x - this->x)*(x - this->x) + (y - this->y)*(y - this->y) <= this->r*this->r;
        }
};

class Cylinder
{
    public:
        float x;
        float y;
        float r;
        float h;
        Cylinder(float x, float y, float r, float h)
        {
            this->x = x;
            this->y = y;
            this->r = r;
            this->h = h;
        }

        bool isInside(float x, float y, float z)
        {
            float dist = sqrt((x - this->x)*(x - this->x) + (y - this->y)*(y - this->y));
            if (dist > this->r)
            {
                return false;
            }
            return true;
        }
};



std::vector<Circle> circles;
std::vector<Cylinder> cylinders;




  
 bool isStateValid(const ob::State *state)
 {

     // cast the abstract state type to the type we expect
     const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
  
     // extract the first component of the state and cast it to what we expect
     const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  
     // extract the second component of the state and cast it to what we expect
     const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    //  return (const void*)rot != (const void*)pos;
  
     // check validity of state defined by pos & rot

    // for (int i = 0; i < circles.size(); i++)
    // {
    //     if (circles[i].isInside(pos->values[0], pos->values[1]))
    //     {
    //         return false;
    //     }
    // }

    if(abs(rot->x) > 0.15 || abs(rot->y) > 0.15)
    {
        return false;
    }

    if (pos->values[2]<0 || pos->values[2]>2)
    {
        return false;
    }

    for (int i = 0; i < cylinders.size(); i++)
    {
        std::vector<double> positions;
        positions.push_back(pos->values[0]);
        positions.push_back(pos->values[1]);
        positions.push_back(pos->values[2]);

        // std::cout << "Checking " << positions[0] << " " << positions[1] << " " << positions[2] << std::endl;

        if (cylinders[i].isInside(positions[0], positions[1], positions[2]))
        {
            return false;
        }
    }

    return true;
  
  
     
 }

 std::vector<double> Quat2Euler(double x, double y, double z, double w)
 {
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    std::vector<double> euler;
    euler.push_back(roll*180/M_PI);
    euler.push_back(pitch*180/M_PI);
    euler.push_back(yaw*180/M_PI);

    return euler;
 }
  
 void plan()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE3StateSpace>());

     std::cout << "A" << std::endl;
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-1);
     bounds.setHigh(1);
  
     space->setBounds(bounds);

     std::cout << "B" << std::endl;
  
     // construct an instance of  space information from this state space
     auto si(std::make_shared<ob::SpaceInformation>(space));
  
     // set state validity checking for this space
     si->setStateValidityChecker(isStateValid);
  
     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();

     
  
     // create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  
     // set the start and goal states
     pdef->setStartAndGoalStates(start, goal);
  
     // create a planner for the defined space
     auto planner(std::make_shared<og::PRM>(si));
  
     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);

     std::cout << "C" << std::endl;
  
     // perform setup steps for the planner
     planner->setup();

     std::cout << "D" << std::endl;
  
  
     // print the settings for this space
     si->printSettings(std::cout);
  
     // print the problem settings
     pdef->print(std::cout);

     std::cout << "E" << std::endl;
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

     std::cout << "F" << std::endl;
  
     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Found solution:" << std::endl;
  
         // print the path to screen
         path->print(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
 }
  
 std::vector<std::vector<double>> planWithSimpleSetup()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE3StateSpace>());

     std::vector<ob::State*> fstates;
     std::vector<std::vector<double>> fstate_db;


  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-10);
     bounds.setHigh(10);
  
     space->setBounds(bounds);
  
     // define a simple setup class
     og::SimpleSetup ss(space);
  
     // set state validity checking for this space
     ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
  
     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 1.0;
    start[3] = 0.0;
    start[4] = 0.0;
    start[5] = 0.0;
    start[6] = 1.0;


  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();
    goal[0] = 5;
    goal[1] = 0;
    goal[2] = 1.0;
    goal[3] = 0.0;
    goal[4] = 0.0;
    goal[5] = 0.0;
    goal[6] = 1.0;
  
     // set the start and goal states
     ss.setStartAndGoalStates(start, goal, 0.3);
  
     // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = ss.solve(5.0);
  
     if (solved)
     {
         std::cout << "Found solution:" << std::endl;
         // print the path to screen
         ss.simplifySolution();
         ss.getSolutionPath().print(std::cout);

        fstates = ss.getSolutionPath().getStates();

        for(int i=0;i<fstates.size();i++)
        {
            const auto *se3state = fstates[i]->as<ob::SE3StateSpace::StateType>();
            const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
            const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            std::vector<double> positions;
            positions.push_back(pos->values[0]);
            positions.push_back(pos->values[1]);
            positions.push_back(pos->values[2]);
            
            std::vector<double> euler = Quat2Euler(rot->x, rot->y, rot->z, rot->w);
            positions.push_back(euler[0]);
            positions.push_back(euler[1]);
            positions.push_back(euler[2]);

            fstate_db.push_back(positions);
        }

        return fstate_db;
        
     }
     else
         std::cout << "No solution found" << std::endl;

    return fstate_db;
 }
  
 int main(int /*argc*/, char ** /*argv*/)
 {
    
    Cylinder c1(2,0,1,10000);
    Cylinder c2(2,1,1,10000);
    Cylinder c3(2,2,1,10000);
    Cylinder c4(2,3,1,10000);
    Cylinder c5(2,-1,1,10000);
    Cylinder c6(2,-2,1,10000);
    Cylinder c7(2,-3,1,10000);

    cylinders.push_back(c1);
    cylinders.push_back(c2);
    cylinders.push_back(c3);
    cylinders.push_back(c4);
    cylinders.push_back(c5);
    cylinders.push_back(c6);
    cylinders.push_back(c7);

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  
    //  plan();
  
     std::cout << std::endl << std::endl;

     std::vector<std::vector<double>> fstate_db = planWithSimpleSetup();
  
     for (int i = 0; i < fstate_db.size(); i++)
     {
        std::cout << 
        boost::format("X: %f, Y: %f, Z: %f, Roll: %f, Pitch: %f, Yaw: %f\n") % fstate_db[i][0] % fstate_db[i][1] % fstate_db[i][2] % fstate_db[i][3] % fstate_db[i][4] % fstate_db[i][5];
     }

    //  write to file
    std::ofstream file;
    file.open("path.txt");
    for (int i = 0; i < fstate_db.size(); i++)
    {
        file << 
        boost::format("%f, %f, %f\n") % fstate_db[i][0] % fstate_db[i][1] % fstate_db[i][2] ;
    }

    file.close();
  
     return 0;
 }