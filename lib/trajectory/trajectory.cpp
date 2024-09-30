#include <Arduino.h>
#include <trajectory.h>
#include <defines.h>
#include <types.h>
#include <i2cFunctions.h>


extern trajectory g_trajectoryQueue[TRAJ_BUFFER_SIZE];
extern uint16_t g_trajEmptySlotIndex;
extern uint8_t g_nTrajSlotsAvailableOnSlave;

void initializeTrajQueue(){
  for(uint16_t i=0;i<TRAJ_BUFFER_SIZE;i++){
    g_trajectoryQueue[i].nPoints=0;
    g_trajectoryQueue[i].type=TRAJTYPE_DEFAULT;
    g_trajectoryQueue[i].id=0;
    for(uint16_t j=0;j<TRAJ_N_POINTS_MAX;j++){
      g_trajectoryQueue[i].position[j]=0;
      g_trajectoryQueue[i].time[j]=0;
    }
    g_trajectoryQueue[i].positionRelativity=RELATIVE_POSITION;
  }
  g_trajEmptySlotIndex=0;
}

void addTrajectoryToQueue(trajectory trajToAdd){
  if(g_trajEmptySlotIndex<TRAJ_BUFFER_SIZE){
    g_trajectoryQueue[g_trajEmptySlotIndex++]=trajToAdd;
  }
}

void removeTrajectoryFromQueue(){
  // implicit : will remove the first item (aka g_trajectoryQueue[0])
  for(uint16_t i=0;i<TRAJ_BUFFER_SIZE-1;i++){
    g_trajectoryQueue[i]=g_trajectoryQueue[i+1];
  }
  g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].nPoints=0;
  g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].type=TRAJTYPE_DEFAULT;
  g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].id=0;
  for(uint16_t j=0;j<TRAJ_N_POINTS_MAX;j++){
    g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].position[j]=0;
    g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].time[j]=0;
  }
  if(g_trajEmptySlotIndex>0)
    g_trajEmptySlotIndex--;
}

void updateTrajectoryQueue(){
  // this assumes a call to i2cGetMotionData has been made prior to calling this function, in order to have an accurate view of the
  // number of slots available in the slave (max 2)
  // One call of this function transmits one point of trajectory through the I2C communication
  static uint8_t nextPointToBeTransmitted;

  if(g_trajEmptySlotIndex==0)
    return; // There is nothing to send
  
  if(g_nTrajSlotsAvailableOnSlave==0)
    return; // There is no slot on the slave to receive a trajectory

  i2cSendTrajPoint( nextPointToBeTransmitted,
                    g_trajectoryQueue[0].nPoints,
                    g_trajectoryQueue[0].id,
                    g_trajectoryQueue[0].type,
                    g_trajectoryQueue[0].position[nextPointToBeTransmitted],
                    g_trajectoryQueue[0].time[nextPointToBeTransmitted],
                    g_trajectoryQueue[0].positionRelativity);
  nextPointToBeTransmitted++;
  if(nextPointToBeTransmitted>=g_trajectoryQueue[0].nPoints){
    removeTrajectoryFromQueue();
    nextPointToBeTransmitted=0;
  } 
}

