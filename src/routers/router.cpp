// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*router.cpp
 *
 *The base class of either iq router or event router
 *contains a list of channels and other router configuration variables
 *
 *The older version of the simulator uses an array of flits and credit to 
 *simulate the channels. Newer version ueses flitchannel and credit channel
 *which can better model channel delay
 *
 *The older version of the simulator also uses vc_router and chaos router
 *which are replaced by iq rotuer and event router in the present form
 */

#include "booksim.hpp"
#include <iostream>
#include <cassert>
#include "router.hpp"

//////////////////Sub router types//////////////////////
#include "iq_router.hpp"
#include "event_router.hpp"
#include "chaos_router.hpp"
///////////////////////////////////////////////////////

int const Router::STALL_BUFFER_BUSY = -2;
int const Router::STALL_BUFFER_CONFLICT = -3;
int const Router::STALL_BUFFER_FULL = -4;
int const Router::STALL_BUFFER_RESERVED = -5;
int const Router::STALL_CROSSBAR_CONFLICT = -6;
//#define ACTION_NUM 6;



Router::Router( const Configuration& config,
		Module *parent, const string & name, int id,
		int inputs, int outputs ) :
TimedModule( parent, name ), _id( id ), _inputs( inputs ), _outputs( outputs ),
   _partial_internal_cycles(0.0)
{
  _crossbar_delay   = ( config.GetInt( "st_prepare_delay" ) + 
			config.GetInt( "st_final_delay" ) );
  _credit_delay     = config.GetInt( "credit_delay" );
  _input_speedup    = config.GetInt( "input_speedup" );
  _output_speedup   = config.GetInt( "output_speedup" );
  _internal_speedup = config.GetFloat( "internal_speedup" );
  _classes          = config.GetInt( "classes" );

  //Added
  action[0]=0;
  action[1]=0;
  action[2]=0;
  action[3]=0;
  prev_flits=0;
  possible_action_num=3;
  throughput=0;
  prev_throughput=0;
  buff=0;
  
#ifdef TRACK_FLOWS
  // cout << "###############################HELLO$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
  _received_flits.resize(_classes, vector<int>(_inputs, 0));
  _stored_flits.resize(_classes);
  _sent_flits.resize(_classes, vector<int>(_outputs, 0));
  _active_packets.resize(_classes);
  _outstanding_credits.resize(_classes, vector<int>(_outputs, 0));
#endif

#ifdef TRACK_STALLS
  _buffer_busy_stalls.resize(_classes, 0);
  _buffer_conflict_stalls.resize(_classes, 0);
  _buffer_full_stalls.resize(_classes, 0);
  _buffer_reserved_stalls.resize(_classes, 0);
  _crossbar_conflict_stalls.resize(_classes, 0);
#endif

}

void Router::AddInputChannel( FlitChannel *channel, CreditChannel *backchannel )
{
  _input_channels.push_back( channel );
  _input_credits.push_back( backchannel );
  channel->SetSink( this, _input_channels.size() - 1 ) ;
}

void Router::AddOutputChannel( FlitChannel *channel, CreditChannel *backchannel )
{
//  cout << "here" << endl;
  _output_channels.push_back( channel );
  _output_credits.push_back( backchannel );
  
  _channel_faults.push_back( false );
  channel->SetSource( this, _output_channels.size() - 1 ) ;
//  cout << "added" << endl;
}

void Router::Evaluate( )
{
  _partial_internal_cycles += _internal_speedup;
  while( _partial_internal_cycles >= 1.0 ) {
    _InternalStep( );
    _partial_internal_cycles -= 1.0;
  }
}

void Router::OutChannelFault( int c, bool fault )
{
  assert( ( c >= 0 ) && ( (size_t)c < _channel_faults.size( ) ) );

  _channel_faults[c] = fault;
}

bool Router::IsFaultyOutput( int c ) const
{
  assert( ( c >= 0 ) && ( (size_t)c < _channel_faults.size( ) ) );

  return _channel_faults[c];
}

/*Router constructor*/
Router *Router::NewRouter( const Configuration& config,
			   Module *parent, const string & name, int id,
			   int inputs, int outputs )
{
  const string type = config.GetStr( "router" );
  Router *r = NULL;
  if ( type == "iq" ) {
    r = new IQRouter( config, parent, name, id, inputs, outputs );
  } else if ( type == "event" ) {
    r = new EventRouter( config, parent, name, id, inputs, outputs );
  } else if ( type == "chaos" ) {
    r = new ChaosRouter( config, parent, name, id, inputs, outputs );
  } else {
    cerr << "Unknown router type: " << type << endl;
  }
  /*For additional router, add another else if statement*/
  /*Original booksim specifies the router using "flow_control"
   *we now simply call these types. 
   */

  return r;
}

// void Router::get_possible_action(double R[100][100], int state, int possible_action[10]){
//     // find R[i][j] value > 0 is action we can do later
//     possible_action_num = 0;
//     if(x<row-1)
//     {
//         possible_action[possible_action_num] = 0;
//         possible_action_num++;
//     }
//     if(x>0)
//     {
//         possible_action[possible_action_num] = 1;
//         possible_action_num++;
//     }
//     if(y<col-1)
//     {
//         possible_action[possible_action_num] = 2;
//         possible_action_num++;
//     }
//     if(y>0)
//     {
//         possible_action[possible_action_num] = 3;
//         possible_action_num++;
//     }
// }

double Router::get_max_q(double Q[6][3], int state){
    double temp_max = 0;
    for(int i=0;i<3;i++)
    {
      if(Q[state][i]>temp_max)
      {
          temp_max = Q[state][i];
      }  
    }
    return temp_max;
}

int Router::inference_best_action(int now_state, double Q[6][3]){
    // get the max value of Q corresponding action when state is nw_state
    double temp_max_q=0;
    int best_action=0;
    for (int i = 0; i < 3; ++i) {
        if (Q[now_state][i] > temp_max_q){
            temp_max_q = Q[now_state][i];
            best_action = i;
        }
    }
    return best_action;
}

int Router::step_iterator(int init_state,double reward)
{   
    double Q_before;//, Q_after;
    // next action
    int next_action=0;
    double max_q;
    srand( (unsigned)time( NULL ) );
    int r_max=2147483647;
    float random_val=rand();
    random_val=random_val/r_max;
    float alpha=0.1;
    float gamma=0.8;
    float epsilon=0.2;
 //   memset(possible_action, 0, 3*sizeof(int));
    //get_possible_action(R, init_state, possible_action);
    possible_action[0]=0;
    possible_action[1]=1;
    possible_action[2]=2;
    // get next action
    if(random_val>epsilon)
    {
     next_action = possible_action[rand()%possible_action_num];
    }
    else
    {
      next_action= inference_best_action(init_state,q_table);
    }
    max_q = get_max_q(q_table, next_action);
    Q_before = q_table[init_state][next_action];
    // update formula Q(s,a)=Q(s,a)+learning rate *(R(s,a)+ gamma * max{Q(s', a')}-Q(s,a))
    q_table[init_state][next_action] = Q_before + alpha *(reward+gamma*max_q-Q_before);
    // Q_after = Q[init_state][next_action];
    action[0]=next_action;
    action[1]=next_action;
    action[2]=next_action;
    action[3]=next_action;
    return next_action;
}

