/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:  Giuseppe Rigano
 * email:   giuseppe.rigano@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#include <string.h>
#include <Kuka.h>

using namespace std;
using namespace XBot;

Kuka::Kuka(const char * config){
 

}

Kuka::~Kuka() {

    
}

void Kuka::init_internal() {

      cout << "Opening FRI Version " 
                << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
                << " Interface for First Sample" << endl;
        {
          // do checks, whether the interface - and the host meets the requirements
          // Note:: This Check remains in friRempte.cpp -- should go to your code ...
          FRI_PREPARE_CHECK_BYTE_ORDER;
          if (!FRI_CHECK_BYTE_ORDER_OK) 
          {
                  cerr << "Byte order on your system is not appropriate - expect deep trouble" <<endl;
          }
          if (!FRI_CHECK_SIZES_OK)
          {
                  cout << "Sizes of datastructures not appropriate - expect even deeper trouble" << endl;

          }
        }
        
                
      lastQuality = FRI_QUALITY_BAD;
      timeCounter=0;
      
      /// perform some arbitrary handshake to KRL -- possible in monitor mode already
      // send to krl int a value
      friInst.setToKRLInt(0,1);
      if ( friInst.getQuality() >= FRI_QUALITY_OK)
      {
              // send a second marker
              friInst.setToKRLInt(0,10);
      }

      //
      // just mirror the real value..
      //
      friInst.setToKRLReal(0,friInst.getFrmKRLReal(1));
      for (int i = 0; i < LBR_MNJ; i++)
      {
          JntVals[i] = friInst.getMsrCmdJntPosition()[i];
      }

  
}

void Kuka::init(){
    
    init_internal();
    
}

int Kuka::recv_from_slave(){
    
    for (int i = 0; i < LBR_MNJ; i++) {
      
         JntVals[i] = friInst.getMsrCmdJntPosition()[i];
     }
     
     return 0;
}

int Kuka::send_to_slave(){
    
    
    friInst.doPositionControl(JntVals);
    // Stop request is issued from the other side
    if ( friInst.getFrmKRLInt(0) == -1) 
    {
            cout << "leaving \n";
            return 1;
    }
    //
    // Quality change leads to output of statistics
    // for informational reasons
    //
    if ( friInst.getQuality() != lastQuality)
    {
            cout << "quality change detected "<< friInst.getQuality()<< " \n";
            cout << friInst.getMsrBuf().intf;
            cout << endl;
            lastQuality=friInst.getQuality();
    }
  
    return 0;
}

////////////////////////////////////
////////////////////////////////////
// SINGLE JOINT PRIVATE FUNCTIONS //
////////////////////////////////////
////////////////////////////////////

bool XBot::Kuka::get_link_pos(int joint_id, double& link_pos)
{

    return false;   
}

bool XBot::Kuka::get_motor_pos(int joint_id, double& motor_pos)
{

    return false;  
}

bool XBot::Kuka::get_link_vel(int joint_id, double& link_vel)
{

    return false;  
}

bool XBot::Kuka::get_motor_vel(int joint_id, double& motor_vel)
{

    return false;  
}

bool XBot::Kuka::get_torque(int joint_id, double& torque)
{

    return false; 
}

bool XBot::Kuka::get_temperature(int joint_id, double& temperature)
{

    return false; 
}

bool XBot::Kuka::get_fault(int joint_id, double& fault)
{

    return false; 
}

bool XBot::Kuka::get_rtt(int joint_id, double& rtt)
{

    return false;   
}

bool XBot::Kuka::get_op_idx_ack(int joint_id, double& op_idx_ack)
{
    

    return false;   
}

bool XBot::Kuka::get_aux(int joint_id, double& aux)
{
    return false;
}

bool XBot::Kuka::get_gains(int joint_id, std::vector< double >& gain_vector)
{
    
    return false; 
}

bool XBot::Kuka::set_pos_ref(int joint_id, const double& pos_ref)
{
    

    return false; 
}

bool XBot::Kuka::set_vel_ref(int joint_id, const double& vel_ref)
{
    
    return false; 
}

bool XBot::Kuka::set_tor_ref(int joint_id, const double& tor_ref)
{
    
    return false; 
}

bool XBot::Kuka::set_gains(int joint_id, const std::vector<double>& gains)
{
    
    return false; 
}
 
bool XBot::Kuka::set_fault_ack(int joint_id, const double& fault_ack)
{
    
   
    return false; 
}

bool XBot::Kuka::set_ts(int joint_id, const double& ts)
{
    

    return false; 
}

bool XBot::Kuka::set_op_idx_aux(int joint_id, const double& op_idx_aux)
{
    
    return false; 
}

bool XBot::Kuka::set_aux(int joint_id, const double& aux)
{
    
    return false; 
}

bool XBot::Kuka::get_ft(int ft_id, std::vector< double >& ft, int channels)
{
    
    return false;   
}


bool XBot::Kuka::get_ft_fault(int ft_id, double& fault)
{
   
    return false;  
}


bool XBot::Kuka::get_ft_rtt(int ft_id, double& rtt)
{

    return false;   
}

bool XBot::Kuka::get_imu(int imu_id, 
                             std::vector< double >& lin_acc, 
                             std::vector< double >& ang_vel, 
                             std::vector< double >& quaternion)
{

    return false;   
}

bool XBot::Kuka::get_imu_fault(int imu_id, double& fault)
{

    return false; 
}

bool XBot::Kuka::get_imu_rtt(int imu_id, double& rtt)
{

    return false; 
}

bool XBot::Kuka::grasp(int hand_id, double grasp_percentage)
{ 
   
    return false; 
}

double XBot::Kuka::get_grasp_state(int hand_id)
{
    
    return -1;   
}
