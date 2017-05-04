#include <cstring>
#include <iostream>
#include <string>
 
using namespace std;
 
#include <sbpl/headers.h>
 
// creating the footprint
void createFootprint(vector<sbpl_2Dpt_t>& perimeter){
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01;
    double halflength = 0.01;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}
 
void initializeEnv(EnvironmentNAVXYTHETALAT* env, 
                   vector<sbpl_2Dpt_t>& perimeter, 
                   char* envCfgFilename, char* motPrimFilename){
    if (!env->InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) {
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
    }
}
 
void setEnvStartGoal(EnvironmentNAVXYTHETALAT* env, 
                     double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta, 
                     int& start_id, int& goal_id){
 
    start_id = env->SetStart(start_x, start_y, start_theta);
    goal_id = env->SetGoal(goal_x, goal_y, goal_theta);
}
 
void initializePlanner(SBPLPlanner* planner, 
                       EnvironmentNAVXYTHETALAT* env,
                       int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution){
    // work this out later, what is bforwardsearch?
    bool bsearch = false;
    planner = new ARAPlanner(env, bsearch);
 
    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}
 
int runPlanner(SBPLPlanner* planner, int allocated_time_secs, 
               vector<int>&solution_stateIDs){
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
 
    if (bRet) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}
 
void writeSolution(EnvironmentNAVXYTHETALAT* env, vector<int> solution_stateIDs,
                   const char* filename){
    std::string discrete_filename(std::string(filename) + std::string(".discrete"));
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }
 
    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env->GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.1);
        cont_y = DISCXY2CONT(y, 0.1);
        cont_theta = DiscTheta2Cont(theta, 16);
        fprintf(fSol_discrete, "%d %d %d\n", x, y, theta);
    }
    fclose(fSol_discrete);
 
    // write the continuous solution to file
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    env->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, 
                                          xythetaPath.at(i).y, 
                                          xythetaPath.at(i).theta);
    }
    fclose(fSol);
}
 
void planxythetalat(char* envCfgFilename, char* motPrimFilename){
    // set the perimeter of the robot   
    vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter);
 
    // initialize an environment
    EnvironmentNAVXYTHETALAT* env = new EnvironmentNAVXYTHETALAT;
    initializeEnv(env, perimeter, envCfgFilename, motPrimFilename);
 
    // specify a start and goal state
    int start_id, goal_id;
    setEnvStartGoal(env, .11, .11, 0, 35, 47.5, 0, start_id, goal_id);
 
    // initialize a planner with start and goal state
    SBPLPlanner* planner = NULL;
    double initialEpsilon = 3.0;
    bool bsearchuntilfirstsolution = false;
    initializePlanner(planner, env, start_id, goal_id, initialEpsilon, 
                      bsearchuntilfirstsolution);
 
    // plan
    vector<int> solution_stateIDs;
    double allocated_time_secs = 10.0; // in seconds
    runPlanner(planner, allocated_time_secs, solution_stateIDs);
 
    // print stats
    env->PrintTimeStat(stdout);
 
    // write out solutions
    std::string filename("sol.txt");
    writeSolution(env, solution_stateIDs, filename.c_str());
 
    delete planner;
}
 
 
int main(int argc, char *argv[])
{
    planxythetalat(argv[1], argv[2]);
}
