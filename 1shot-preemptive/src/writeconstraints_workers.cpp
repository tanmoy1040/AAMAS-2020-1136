/* License */
/*
Copyright (c) <2014>, <Indranil Saha>
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include "primitive.h"
#include "readinputs.h"
#include "writeconstraints_workers.h"

void declareVariables_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1, count2;
  for (count1 = 0; count1 < workspace.number_of_wrobs; count1++)
  {
    for (count2 = 0; count2 < workspace.hlen; count2++)
    {
        ofp << "(declare-const wtraj_" << count1 + 1 << "_" << count2 + 1 <<" Int)" << endl;
    }
    ofp << endl;
  }
  ofp << endl;
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
        ofp << "(declare-const wprim_" << count1 << "_" << count2 <<" Int)" << endl;
    }
    ofp << endl;
  }
  ofp << endl;  
  for (count1 = 0; count1 < workspace.number_of_wrobs; count1++)
  {
    for (count2 = 0; count2 < workspace.hlen; count2++)
    {
        ofp << "(declare-const wx_" << count1 + 1 << "_" << count2 + 1 <<" Int)" << endl;
    }
    ofp << endl;
  }
  ofp << endl;
  for (count1 = 0; count1 < workspace.number_of_wrobs; count1++)
  {
    for (count2 = 0; count2 < workspace.hlen; count2++)
    {
        ofp << "(declare-const wy_" << count1 + 1 << "_" << count2 + 1 <<" Int)" << endl;
    }
    ofp << endl;
  }
  ofp << endl;
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
        ofp << "(declare-const ch_" << count1 << "_" << count2 << " Int)" << endl;
    }
    ofp << endl;
  }
  ofp << endl;
  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
     ofp << "(declare-const waitcount_" << count1 << "_" << count2 << " Int)" << endl;
    }
  }
  ofp << endl;
  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
     ofp << "(declare-const rechcount_" << count1 << "_" << count2 << " Int)" << endl;
    }
  }
  ofp << endl;
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++) //change15
  {
    for (count2 = 1; count2 <= workspace.hlen-1; count2++)
    {
      ofp << "(declare-const rechassigned_" << count1 << "_" << count2 << " Int)" << endl;
    }
  }
  ofp << endl;
  
  ofp << "(declare-const total_rech" << " Int)" << endl;
  ofp << "(declare-const total_wait" << " Int)" << endl;
}


void instantiateVariables_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1, count2;

  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
     ofp << "(assert (= wtraj_" << count1 << "_1 1))" << endl;
  }
  ofp << endl;
  /* trajectory of workers during hypercycle */
  for (count1 = 0; count1 < workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 < workspace.hlen; count2++)
    {
	ofp << "(assert (and (>= wtraj_" << count1+1 << "_" << count2+1 << " 1) ";
    	ofp << "(<= wtraj_" << count1+1 << "_" << count2+1 << " " << workers[count1].looplen << ")))" << endl;
    }
    ofp << endl;
  }
 
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen-1; count2++)
    {
        ofp << "(assert (and (>= wprim_" << count1 << "_" << count2 << " 0) ";
        ofp << "(<= wprim_" << count1 << "_" << count2 << " 2)))" << endl;
    }
  }

  /* X-coordinate of workers during hypercycle */
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
        ofp << "(assert (and (>= wx_" << count1 << "_" << count2 << " " << workspace.min_x << ") ";
        ofp << "(<= wx_" << count1 << "_" << count2 << " " << workspace.length_x << ")))" << endl;
    }
    ofp << endl;
  }
  ofp << endl;

  /* Y-coordinate of workers during hypercycle */
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
        ofp << "(assert (and (>= wy_" << count1 << "_" << count2 << " " << workspace.min_y << ") ";
        ofp << "(<= wy_" << count1 << "_" << count2 << " " << workspace.length_y << ")))" << endl;
    }
    ofp << endl;
  }
  ofp << endl;

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
     ofp << "(assert (= ch_" << count1 << "_1 " << workers[count1-1].fch << "))" << endl;
  }
  ofp << endl;

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
        ofp << "(assert (and (>= ch_" << count1 << "_" << count2 << " 0) ";
        ofp << "(<= ch_" << count1 << "_" << count2 << " " << workers[count1-1].fch << ")))" << endl;
    }
    ofp << endl;
  }
  ofp << endl;

  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
      ofp << "(assert (and (>= waitcount_" << count1 << "_" << count2 << " 0) ";
      ofp << "(<= waitcount_" << count1 << "_" << count2 << " 1)))" << endl;
    }
  }
  ofp << endl;

  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
      ofp << "(assert (and (>= rechcount_" << count1 << "_" << count2 << " 0) " << endl;
      ofp << "(<= rechcount_" << count1 << "_" << count2 << " 1)))" << endl;
    }
  }
  ofp << endl;

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++) //change15
  {
    for (count2 = 1; count2 <= workspace.hlen-1; count2++)
    {
      ofp << "(assert (and (>= rechassigned_" << count1 << "_" << count2 << " 1) " << endl;
      ofp << "(<= rechassigned_" << count1 << "_" << count2 << " " << workspace.number_of_rechs << ")))" << endl;
    }
  }
  ofp << endl;
}


void writeMatchingConstraints_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1;

  /* Same location at first and last point of the hypercycle */
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    ofp << "(assert (and (= wx_" << count1 << "_1 wx_" << count1 << "_" << workspace.hlen << ") ";
    ofp << "(= wy_" << count1 << "_1 wy_" << count1 << "_" << workspace.hlen << ")";
    ofp << "))"<< endl;
  }
  ofp << endl;
  /* Same charge at first and last point of the hypercycle */
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    ofp << "(assert (and (= ch_" << count1 << "_1 ch_" << count1 << "_" << workspace.hlen << ")))" << endl; // change12
  }
  ofp << endl;
}


void writeTrajectoryToLoopMapping_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1, count2, count3;

  ofp << "; mapping : Worker Trajectory to Loop point" << endl;
  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<=workspace.hlen; count2++)
    {
      for(count3=1; count3<=workers[count1-1].looplen; count3++)
      {
	 ofp << "(assert (=> (= wtraj_" << count1 << "_" << count2 << " " << count3 << ") " << endl;
	 ofp << "(and (= wx_" << count1 << "_" << count2 << " " << workers[count1-1].looptraj[count3-1].x << ") ";
	 ofp << "(= wy_" << count1 << "_" << count2 << " " << workers[count1-1].looptraj[count3-1].y << "))))" << endl << endl;
      }
    }
  }
}


void placeRecharger1 (ofstream &ofp, worker_vec_t workers, unsigned int w_id, unsigned int timept, unsigned int r_id, int ofs_x, int ofs_y) //change15
{
  ofp << "  (and (= rprim_" << r_id << "_" << timept << " 1) (= rechassigned_" << w_id << "_" << timept << " " << r_id << ")" << endl;
  ofp << "   (= rx_" << r_id << "_" << timept << " (+ wx_" << w_id << "_" << timept << " " << ofs_x << "))" << endl;
  ofp << "   (= ry_" << r_id << "_" << timept << " (+ wy_" << w_id << "_" << timept << " " << ofs_y << ")))" << endl;
}


void writeTransition_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1, count2, count3;
  worker_t worker;
  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for (count2=1; count2<=workspace.hlen-1; count2++)
    {
      ofp << "(declare-const rech_amt_" << count1 << "_" << count2 << " Int)" << endl;
    }
  }
  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for (count2=1; count2<=workspace.hlen-1; count2++)
    {
      ofp << "(assert (and (<= rech_amt_" << count1 << "_" << count2 <<  " " << workspace.recharge << ") ";
      ofp << "(> rech_amt_" << count1 << "_" << count2 << " 0)))" << endl; //change11
    }
  }

  /* intermediate recharging */
  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    ofp << "(declare-const fch_" << count1 << " Int)" << endl;
    ofp << "(assert (= fch_" << count1 << " " << workers[count1-1].fch << "))" << endl;
    worker = workers[count1-1];
    for(count2=1; count2<=workspace.hlen-1; count2++)
    {
      /* changes in transition and charge for a MOTION primitive */
      ofp << "; wprim = 1" << endl;
      ofp << "(assert (=> (and (< wtraj_" << count1 << "_" << count2 << " " << worker.looplen << ") ";
      ofp << "(= wprim_" << count1 << "_" << count2 << " 1)) " << endl;
      ofp << "(and (= wtraj_" << count1 << "_" << count2+1 << " (+ wtraj_" << count1 << "_" << count2 << " 1)) ";
      ofp << "(= ch_" << count1 << "_" << count2+1 << " ";
      //ofp << "(- ch_" << count1 << "_" << count2 << " " << workspace.discharge << ")))))" << endl << endl;
      ofp << "(- ch_" << count1 << "_" << count2 << " " << worker.req_charge[(count2-1) % worker.looplen] << ")))))" << endl << endl;

      // circularly returns to the first position
      ofp << "(assert (=> (and (= wtraj_" << count1 << "_" << count2 << " " << worker.looplen << ") ";
      ofp << "(= wprim_" << count1 << "_" << count2 << " 1))" << endl;
      ofp << "(and (= wtraj_" << count1 << "_" << count2+1 << " 1) ";
      ofp << "(= ch_" << count1 << "_" << count2+1 << " ";
      //ofp << "(- ch_" << count1 << "_" << count2 << " " << workspace.discharge << ")))))" << endl << endl;
      ofp << "(- ch_" << count1 << "_" << count2 << " " << worker.req_charge[(count2-1) % worker.looplen] << ")))))" << endl << endl;

      /* changes in transition and charge for a STATIC primitive */
      ofp << "; wprim = 0" << endl;
      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 0) " << endl;
      ofp << "(and (= wtraj_" << count1 << "_" << count2+1 << " wtraj_" << count1 << "_" << count2 << ") ";
      ofp << "(= ch_" << count1 << "_" << count2+1 << " ch_" << count1 << "_" << count2 << "))))" << endl << endl;
    }
  }

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    worker = workers[count1-1];
    for (count2 = 1; count2 <= workspace.hlen-1; count2++)
    {
      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 2)" << endl;
      ofp << "(and (= wtraj_" << count1 << "_" << count2+1 << " wtraj_" << count1 << "_" << count2 << ") " << endl;
      ofp << "(< ch_" << count1 << "_" << count2 << " " << worker.fch << ")" << endl; //change 30jan2019
      ofp << "(= ch_" << count1 << "_" << count2+1 << " ";
      ofp << "(+ ch_" << count1 << "_" << count2 << " ";
      ofp << "rech_amt_" << count1 << "_" << count2 << "))" << endl;

      ofp << "(or" << endl; //change15
      for (count3 = 1; count3 <= workspace.number_of_rechs; count3++)
      {
        placeRecharger1 (ofp, workers, count1, count2, count3,  1,  0);
        placeRecharger1 (ofp, workers, count1, count2, count3,  1,  1);
        placeRecharger1 (ofp, workers, count1, count2, count3,  0,  1);
        placeRecharger1 (ofp, workers, count1, count2, count3, -1,  1);
        placeRecharger1 (ofp, workers, count1, count2, count3, -1,  0);
        placeRecharger1 (ofp, workers, count1, count2, count3, -1, -1);
        placeRecharger1 (ofp, workers, count1, count2, count3,  0, -1);
        placeRecharger1 (ofp, workers, count1, count2, count3,  1, -1);
      } 
      ofp << "))))" << endl << endl;
    }
  }
  ofp << endl;
}


void writeTransition_workers2 (ofstream &ofp, workspace_t workspace, worker_vec_t workers, workerhalt_vec_t workerhalt, unsigned int max_timept)
{
  unsigned int count1, count2, count3;
  worker_t worker;
  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for (count2=workerhalt[count1-1].timept; count2<=max_timept-1; count2++)
    {
      ofp << "(declare-const rech_amt_" << count1 << "_" << count2 << " Int)" << endl;
    }
  }

  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for (count2=workerhalt[count1-1].timept; count2<=max_timept-1; count2++)
    {
      ofp << "(assert (and (<= rech_amt_" << count1 << "_" << count2 <<  " " << workspace.recharge << ") ";
      ofp << "(> rech_amt_" << count1 << "_" << count2 << " 0)))" << endl; //change11
    }
  }

  /* intermediate recharging */
  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    ofp << "(declare-const fch_" << count1 << " Int)" << endl;
    ofp << "(assert (= fch_" << count1 << " " << workers[count1-1].fch << "))" << endl;
    worker = workers[count1-1];
    for (count2=workerhalt[count1-1].timept; count2<=max_timept-1; count2++)
    {
      /* changes in transition and charge for a STATIC primitive */
      ofp << "; wprim = 0" << endl;
      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 0) " << endl;
      ofp << "(= ch_" << count1 << "_" << count2+1 << " ch_" << count1 << "_" << count2 << ")))" << endl << endl;
    }
  }

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    worker = workers[count1-1];
    for (count2=workerhalt[count1-1].timept; count2<=max_timept-1; count2++)
    {
      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 2)" << endl;
      ofp << "(and" << endl;
      ofp << "(< ch_" << count1 << "_" << count2 << " " << worker.fch << ")" << endl; //change 30jan2019
      ofp << "(= ch_" << count1 << "_" << count2+1 << " ";
      ofp << "(+ ch_" << count1 << "_" << count2 << " ";
      ofp << "rech_amt_" << count1 << "_" << count2 << "))" << endl;

      ofp << "(or" << endl; //change15
      for (count3 = 1; count3 <= workspace.number_of_rechs; count3++)
      {
        placeRecharger1 (ofp, workers, count1, count2, count3,  1,  0);
        placeRecharger1 (ofp, workers, count1, count2, count3,  1,  1);
        placeRecharger1 (ofp, workers, count1, count2, count3,  0,  1);
        placeRecharger1 (ofp, workers, count1, count2, count3, -1,  1);
        placeRecharger1 (ofp, workers, count1, count2, count3, -1,  0);
        placeRecharger1 (ofp, workers, count1, count2, count3, -1, -1);
        placeRecharger1 (ofp, workers, count1, count2, count3,  0, -1);
        placeRecharger1 (ofp, workers, count1, count2, count3,  1, -1);
      } 
      ofp << "))))" << endl << endl;
    }
  }
  ofp << endl;
}


void minimizeWaiting_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1, count2;
  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 0) ";
      ofp << "(and (= waitcount_" << count1 << "_" << count2 << " 1) ";
      ofp << "(= rechcount_" << count1 << "_" << count2 << " 0))))" << endl;

      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 2) ";
      ofp << "(and (= waitcount_" << count1 << "_" << count2 << " 1) "; // change16 -> 1 to 0
      ofp << "(= rechcount_" << count1 << "_" << count2 << " 1))))" << endl;

      ofp << "(assert (=> (= wprim_" << count1 << "_" << count2 << " 1) ";
      ofp << "(and (= waitcount_" << count1 << "_" << count2 << " 0) " << endl;
      ofp << "(= rechcount_" << count1 << "_" << count2 << " 0))))" << endl;
    }
    ofp << endl << endl;
  }
  /* minimize */
  ofp << "(assert (= total_wait (+ ";
  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
       ofp << "waitcount_" << count1 << "_" << count2 << endl;
    }
  } 
  ofp << ")))" << endl << endl;

  ofp << "(assert (= total_rech (+ ";
  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  { 
    for(count2=1; count2<workspace.hlen; count2++)
    {  
       ofp << "rechcount_" << count1 << "_" << count2 << endl;
    }
  }
  ofp << ")))" << endl << endl;

  ofp << "(assert (= total_rcost (+ ";
  for (count1=1; count1<=workspace.number_of_rechs; count1++)
  {
    for(count2=1; count2<workspace.hlen; count2++)
    {
      ofp << "rcost_" << count1 << "_" << count2 << endl;
    }
  }
  ofp << ")))" << endl << endl;

  ofp << "(minimize total_wait)" << endl;
  ofp << "(check-sat)" << endl;
  ofp << "(get-objectives)" << endl;
}


void writeOutputConstraints_workers (ofstream &ofp, workspace_t workspace, worker_vec_t workers)
{
  unsigned int count1, count2;

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen-1; count2++)
    {
	ofp << "(get-value (wprim_" << count1 << "_" << count2 << ")) " << endl;
    }
    ofp << endl;
  }
 
  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
	ofp << "(get-value (wtraj_" << count1 << "_" << count2 << ")) " << endl;
    }
    ofp << endl;
  }

  for (count1=1; count1<=workspace.number_of_wrobs; count1++)
  {
    for (count2=1; count2<=workspace.hlen; count2++)
    {
      ofp << "(get-value (wx_" << count1 << "_" << count2 << "))" << endl;
      ofp << "(get-value (wy_" << count1 << "_" << count2 << "))" << endl;
    }
    ofp << endl;
  }

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++)
  {
    for (count2 = 1; count2 <= workspace.hlen; count2++)
    {
        ofp << "(get-value (ch_" << count1 << "_" << count2 << ")) " << endl;
    }
    ofp << endl;
  }

  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  { 
    for(count2=1; count2<workspace.hlen; count2++)
    {  
       ofp << "(get-value (rechcount_" << count1 << "_" << count2 << "))" << endl;
    }
  }

  for(count1=1; count1<=workspace.number_of_wrobs; count1++)
  { 
    for(count2=1; count2<workspace.hlen; count2++)
    {  
       ofp << "(get-value (waitcount_" << count1 << "_" << count2 << "))" << endl;
    }
  }

  for (count1 = 1; count1 <= workspace.number_of_wrobs; count1++) //change15
  {
    for (count2 = 1; count2 <= workspace.hlen-1; count2++)
    {
      ofp << "(get-value (rechassigned_" << count1 << "_" << count2 << "))" << endl;
    }
  }

  ofp << "(get-value (total_rech))" << endl;
}

