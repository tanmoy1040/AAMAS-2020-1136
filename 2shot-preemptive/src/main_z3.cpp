#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include "primitive.h"
#include "readinputs.h"
#include "writeconstraints_recharger.h"
#include "writeconstraints_workers.h"
#include "writeconstraints_recharger_optimal.h"
using namespace std;

string printTimeDifference(double wcts, double wcte)
{
  double duration;
  int hr, min;
  double sec;
  stringstream ss;

  duration = wcte - wcts;
  hr = duration / 3600; 
  min = (duration / 60) - (hr * 60) ;
  sec = duration - hr * 3600 - min * 60;
  cout << endl << hr << "h " << min << "m " << (((sec - int(sec)) > 0.5) ? (int(sec) + 1) : int(sec)) << "s" << endl;
  ss << hr << "h " << min << "m " << (((sec - int(sec)) > 0.5) ? (int(sec) + 1) : int(sec)) << "s";
  string t = ss.str();
  return t;
}


int is_sat (string filename)
{
 ifstream ifp;
 int flag = 2;
 string str;
 ifp.open(filename.c_str());

 string line, line1 = "CUT";
  
 if(ifp.is_open())
 {
   while (getline(ifp, line))
   {
     if (line.compare("sat")==0)
      flag=1;
     if (line.compare("unsat")==0)
     {
       flag=0;
       break;
     }
   }
 }
 ifp.close();
 return flag;
}


unsigned int get_max_intrech_timept (workerhalt_vec_t workerhalt)
{
  unsigned int max_intrech_timept = 0;
  
  for (unsigned int count = 0; count < workerhalt.size(); count++)
  {
    unsigned int d = workerhalt[count].timept;
    if (d > max_intrech_timept)
    {
      max_intrech_timept = d;
    }
  }
  return max_intrech_timept;
}


unsigned int get_max_intrech_duration (workerhalt_vec_t workerhalt)
{
  unsigned int max_intrech_duration = 0;
  
  for (unsigned int count = 0; count < workerhalt.size(); count++)
  {
    unsigned int d = workerhalt[count].duration;
    if (d > max_intrech_duration)
    {
      max_intrech_duration = d;
    }
  }
  return max_intrech_duration;
}


void generateZ3File (prim_vec_t primitives, pos_vec_t obstacles, workspace_t workspace, worker_vec_t workers, unsigned int max_timept)
{ 
  ofstream ofp;
  ofp.open("constraints.smt2");

  declareVariables_workers (ofp, workspace, workers);
  declareVariables_rechargers (ofp, workspace);

  instantiateVariables_workers (ofp , workspace, workers);
  writeTrajectoryToLoopMapping_workers (ofp, workspace, workers);
  writeMatchingConstraints_workers (ofp, workspace, workers);
  writeTransition_workers (ofp, workspace, workers);

  writeTransitionConstraints_rechargers (ofp, primitives, obstacles, workspace, max_timept);
  writeCollisionAvoidanceConstraints (ofp, primitives, workspace);

  minimizeWaiting_workers (ofp, workspace, workers);
  writeOutputConstraints_rechargers (ofp, workspace);
  writeOutputConstraints_workers (ofp, workspace, workers);

  ofp.close();
}


void generateZ3File2 (prim_vec_t primitives, pos_vec_t obstacles, workspace_t workspace, worker_vec_t workers, rechinst_vec_t rechinstances, workerhalt_vec_t workerhalt, unsigned int max_timept, unsigned int ext_hyp_start)
{
  ofstream ofp;
  ofp.open("constraints2.smt2");

  declareVariables2 (ofp, workspace, max_timept);
  writeWaypointsConstraints2 (ofp, workspace, obstacles, workers, rechinstances, workerhalt, max_timept);
  writeTransitionConstraints_rechargers (ofp, primitives, obstacles, workspace, max_timept);
  writeCollisionAvoidanceConstraints2 (ofp, workspace, workers, max_timept);
  //minimize_rcost (ofp, max_timept, ext_hyp_start);
  writeOutputConstraints2 (ofp, max_timept, workspace, workerhalt, ext_hyp_start);
}


int limit_found (int* bound_flag, position* range, unsigned int rcost)
{
  unsigned int count1=-999, count2=999, found_down=0, found_up=0, found_flag=0;
  for(count1=1; count1<rcost; count1++)
  {
    if(bound_flag[count1]==1 && (bound_flag[count1+1]==3 || bound_flag[count1+1]==2))
    { 
      found_down=1;
      found_flag=1;
      break;
    }
  }
  for(count2=rcost; count2>1; count2--)
  {
    if(bound_flag[count2]==2 && (bound_flag[count2-1]==3 || bound_flag[count2-1]==1))
    {
      found_up=1;
      found_flag=2;
      break;
    }
  }
  if (found_up && found_down)
  {
    (*range).x=count2; (*range).y=count1;
    found_flag=3;
  }
  return found_flag;
}

/*
position optimize_rcost_binary (prim_vec_t primitives, pos_vec_t obstacles, workspace_t workspace, worker_vec_t workers, rechinst_vec_t rechinstances, workerhalt_vec_t workerhalt, unsigned int max_timept, unsigned int rcost, string filename)
{
  stringstream ss;
  position range;
  unsigned int upper, middle, lower, sat_res, pivot;
  upper=middle=pivot=rcost; lower=1;
  int bound_flag[rcost+1];
  for(unsigned int count=1; count <=rcost; count++)
  {
    bound_flag[count]=0;
  }
  ss.str("");
  ss << "timeout 30s z3 ../examples/constraints2.smt2 > ../examples/my_output/z3_output_" << workspace.number_of_wrobs << "_" << workspace.hlen << "_ext1";
  string cmdline = ss.str();

  cout << "FINDING OPTIMAL RANGE OF RECHARGER COST :" << endl;
  for(int i=1; i<=2; i++)
  {
    while (upper-lower > 1)
    {
      generateZ3File2 (primitives, obstacles, workspace, workers, rechinstances, workerhalt, max_timept, middle);
      system (cmdline.c_str());
      sat_res = is_sat(filename);

      if(sat_res==0)
      {
        //cout << "Middle = " << middle << " is an UNSAT point." << endl;
        lower = middle;
        if(i==1) range.y = lower;
        bound_flag[middle] = 1;
        int found_flag = limit_found(bound_flag, &range, rcost);
        if (found_flag==3) break;
      }
      else if(sat_res==1)
      {
        //cout << "Middle = " << middle << " is a SAT point." << endl;
        upper = middle;
        if(i==2) range.x = upper;
        bound_flag[middle] = 2;
        int found_flag = limit_found(bound_flag, &range, rcost);
        if (found_flag==3) break;
      }
      else
      {
        //cout << "Middle = " << middle << " is an UNKNOWN point." << endl;
        bound_flag[middle] = 3;
        int found_flag = limit_found(bound_flag, &range, rcost);
        if (found_flag==0) 
        {
          upper=middle;
          if(i==1) pivot=middle;
        }
        else if (found_flag==1) lower=middle;
        else if (found_flag==2) upper=middle;
        else if (found_flag==3) break;
      }
      middle = (lower + upper)/2;
    }
    lower=pivot; upper=rcost;
  }
  generateZ3File2 (primitives, obstacles, workspace, workers, rechinstances, workerhalt, max_timept, range.x);
  system (cmdline.c_str());

  return range;
}
*/


unsigned int total_ext_hyp (unsigned int ext_hyp, workerhalt_vec_t workerhalt)
{
  unsigned int ext_hyp_new = ext_hyp;
  for (unsigned int count=0; count<workerhalt.size(); count++)
  {
    if (workerhalt[count].duration > 1)
    {
      ext_hyp_new += workerhalt[count].duration - 1;
    }
  }
  return ext_hyp_new;
}

unsigned int get_total_lastrech_dur (workerhalt_vec_t workerhalt)
{
  unsigned int tot_dur=0;
  for (unsigned int count=0; count<workerhalt.size(); count++)
  {
    tot_dur = tot_dur + workerhalt[count].duration;
  }
  return tot_dur;
}

unsigned int get_tot_time_after_intrech (workerhalt_vec_t workerhalt, unsigned int tot_ext)
{
  unsigned int tot_dur=0;
  for (unsigned int count=0; count < workerhalt.size(); count++)
  {
    tot_dur = tot_dur + (tot_ext - workerhalt[count].timept + 1);
  }
  cout << endl << "tot time after int. rech " << tot_dur << endl;
  return tot_dur;
}

unsigned int get_total_extra_halt_time (workerhalt_vec_t workerhalt, unsigned int ext_hyp_init)
{
  unsigned int tot_dur = 0;
  for (unsigned int count=0; count < workerhalt.size(); count++)
  {
    tot_dur = tot_dur + (ext_hyp_init - workerhalt[count].timept);
  }
  return tot_dur;
}

void writeRobTrajectories (worker_vec_t workers, string filename1, string filename2, unsigned int nrechs, testcase_t &testcase)
{
  stringstream ss_wtrajs, ss_rtrajs;
  ss_wtrajs << "-----------------------Workers------------------------" << endl;
  ss_wtrajs << writeWorkingTraj(workers) << endl; //workers' trajectories
  testcase.consolid_wtrajs = ss_wtrajs.str();

  ss_rtrajs << "----------------------Rechargers-----------------------" << endl;
  for (unsigned int count = 1; count <= nrechs; count++)
  {
    ss_rtrajs << getRechargerTraj(filename2, count);
  }
  testcase.consolid_rtrajs = ss_rtrajs.str();
  cout << testcase.consolid_wtrajs << testcase.consolid_rtrajs << endl;
}


int main ()
{
  double wcts, wcte;
  struct timeval tm;
  gettimeofday( &tm, NULL );
  wcts = (double)tm.tv_sec + (double)tm.tv_usec * .000001;

  prim_vec_t primitives;
  pos_vec_t obstacles;
  workspace_t workspace;
  worker_vec_t workers;
  rechinst_vec_t rechinstances;
  workerhalt_vec_t workerhalt;
  string filename, filename1, filename2, cmdline;
  stringstream ss;
  testcase_t testcase;

  cout << endl << "============================ TWO-SHOT (preemptive) ALGORITHM ===========================" << endl;
  cout << "Reading inputs ..." << endl;
  readPrimitives (primitives);
  readObstacles (obstacles);
  readWorkspace (workspace, obstacles, workers);

  workspace.min_x = 0; 
  workspace.min_y = 0; 

  cout << "Processing, please wait ..." << endl;

  // FIRST phase
  generateZ3File (primitives, obstacles, workspace, workers, workspace.hlen);
  ss << "time timeout 15000s z3 ../examples/constraints.smt2 > ../examples/my_output/z3_output_" << workspace.number_of_wrobs << "_" << workspace.hlen;
  cmdline = ss.str();
  system (cmdline.c_str());
    
  filename = cmdline.substr (cmdline.find("../examples/my_output"));
  filename1 = filename;

  int sat_res = is_sat(filename1);
  if(sat_res==0)
  {  
     cout << " --> UNSAT" << endl;
     ss.str(""); ss.clear();
     ss << "rm " << filename1;
     system(ss.str().c_str());
  }
  else if(sat_res==1)
  {
     cout << " --> SAT" << endl;
  }
  else
  {
     cout << " --> UNKNOWN" << endl;
     exit(0);
  }
  cout << endl << "SAT testing IS DONE ..." << endl << endl;
  cout << "Intermediate recharging instances are as below :" << endl;
  cout << "-------------------------------------------------------" << endl;

  // fetching outcomes from first phase to use them in the second phase
  for (unsigned int count=0; count<workers.size(); count++)
  {
    ss.str(""); ss << "((wx_" << count+1 << "_"; string robstrx = ss.str();
    ss.str(""); ss << "((wy_" << count+1 << "_"; string robstry = ss.str();
    workers[count].workingtraj = readWorkingTraj (filename1, robstrx, robstry, workspace.hlen); // change8
  }

/*========================================================================================================== */

  for (unsigned int count1 = 0; count1 <  workspace.number_of_rechs; count1++)
  {
    workspace.rpos_start.push_back(readRechargerPosition (filename1, count1+1 , 1));
    workspace.rpos_hlen.push_back(readRechargerPosition (filename1, count1+1, workspace.hlen));
  }
  testcase.rechinstances = readRechInstances (rechinstances, workers, filename1);
  testcase.lastrechs = readWorkerHaltDetails (workerhalt, workspace, workers, filename1);

  // SECOND phase
  unsigned int ext_hyp, upper = workspace.hlen + 50, result;
  ss.str("");
  ss << "timeout 4000s z3 ../examples/constraints2.smt2 > ../examples/my_output/z3_output_" << workspace.number_of_wrobs << "_" << workspace.hlen << "_ext1";
  cmdline = ss.str();
  filename = cmdline.substr (cmdline.find("../examples/my_output"));
  filename2 = filename;
  
  unsigned int ext_hyp_init = workspace.hlen;  // for calculation
  unsigned int ext_hyp_start = get_max_intrech_timept (workerhalt) + get_max_intrech_duration (workerhalt);
  ext_hyp = ext_hyp_start;
  // ext_hyp = workspace.hlen;
  
  for ( ; ext_hyp <= upper ; ext_hyp++)
  {
    generateZ3File2 (primitives, obstacles, workspace, workers, rechinstances, workerhalt, ext_hyp, ext_hyp_start);
    system (cmdline.c_str());
    result = is_sat(filename2);

    if(result==1) 
    {
      cout << " --> SAT !!" << endl;
      break;
    }
    else if(result==0)
    {
       cout << " --> UNSAT" << endl;
       ss.str(""); ss.clear();
       ss << "rm " << filename2;
       system(ss.str().c_str());
       continue;
    }
    else 
    {
       cout << " --> UNKNOWN" << endl;
    }
  }
  cout << endl;

/*========================================================================================================== */

  // reading output
  writeRobTrajectories (workers, filename1, filename2, workspace.number_of_rechs, testcase);
  cout << endl << "EXTENDED HYPERLENGTH IS : " << ext_hyp << endl;
  //unsigned int rcost = readValue ("((total_rcost ", filename2); 
  //cout << endl << "Unoptimized cost of recharger: " << rcost << endl;
  unsigned int tot_wt_upto_halt = tot_wait_upto_halt ("((waitcount", filename1, ext_hyp_init, workspace.number_of_wrobs);
  testcase.ext_hlen = ext_hyp;
  testcase.intrechcount = rechinstances.size();
  testcase.lastrechcount = get_total_lastrech_dur (workerhalt);

  // wait
  unsigned int idle_upto_halt = tot_wt_upto_halt; // including intrechs
  unsigned int idle_after_halt = (ext_hyp-ext_hyp_init) * workspace.number_of_wrobs; // including lastrechs

  // hard effic  
  unsigned int tot_hard_wait = idle_upto_halt + idle_after_halt; // including recharge instances
  double hardeffic = 100*((workspace.number_of_wrobs*ext_hyp) - tot_hard_wait) / (double) (workspace.number_of_wrobs*ext_hyp);
  testcase.hardeffic = hardeffic;
  cout << "EFFICIENCY (recharging counted as waiting): " << hardeffic << endl;

  // soft effic
  unsigned int tot_soft_wait = idle_upto_halt + idle_after_halt - testcase.intrechcount - testcase.lastrechcount;
  double softeffic = 100*((workspace.number_of_wrobs*ext_hyp) - tot_soft_wait) / (double) (workspace.number_of_wrobs*ext_hyp);
  testcase.softeffic = softeffic;
  cout << "EFFICIENCY (recharging NOT counted as waiting): " << softeffic << endl << endl;

  testcase.wait_idle = tot_soft_wait;  // recharging not considered
  cout << "Wait_idle for workers : " << testcase.wait_idle << endl;
  cout << "Total recharge : " << testcase.intrechcount + testcase.lastrechcount << endl;
  testcase.dim = workspace.length_x;
  testcase.wcount = workspace.number_of_wrobs;
  testcase.rcount = workspace.number_of_rechs;
  testcase.unitrech = workspace.recharge;
  testcase.orig_hlen = workspace.hlen;

  cout << "-------------------------------------------------------";
 
  gettimeofday( &tm, NULL );
  wcte = (double)tm.tv_sec + (double)tm.tv_usec * .000001;
  testcase.elapsed_time = printTimeDifference(wcts, wcte); cout << endl;
  return 0;
}
