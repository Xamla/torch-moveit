--- LUA wrapper for moveit planning environment
-- dependency to tourch.ros
-- @classmod Plan
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local gnuplot = require 'gnuplot'

local Plan = torch.class('moveit.Plan', moveit)

local f

function init()
  local Plan_method_names = {
    "new",
    "delete",
    "release",
    "getStartStateMsg",
    "getTrajectoryMsg",
    "getPlanningTime",
    "plot",
    "convertTrajectoyMsgToTable",
    "convertStartStateMsgToTensor"
  }
  f = utils.create_method_table("moveit_Plan_", Plan_method_names)
end

init()

function Plan:__init()
  self.o = f.new()
  self.moveit_msgs_RobotStateSpec = ros.get_msgspec('moveit_msgs/RobotState')
  self.moveit_msgs_RobotTrajectory = ros.get_msgspec('moveit_msgs/RobotTrajectory')
end

function Plan:cdata()
  return self.o
end

function Plan:release()
  f.release(self.o)
end

---Create a ros message for the robot state: moveit_msgs/RobotState
--@tparam[opt] ros.Message result
--@treturn ros.Message
function Plan:getStartStateMsg(result)
  local msg_bytes = torch.ByteStorage()
  f.getStartStateMsg(self.o, msg_bytes:cdata())
  local msg = result or ros.Message(self.moveit_msgs_RobotStateSpec, true)
  msg:deserialize(msg_bytes)
  return msg
end

---Create a ros message for the robot trajectory: moveit_msgs/RobotTrajectory
--@tparam[opt] ros.Message result
--@treturn ros.Message
function Plan:getTrajectoryMsg(result)
  local msg_bytes = torch.ByteStorage()
  f.getTrajectoryMsg(self.o, msg_bytes:cdata())
  local msg = result or ros.Message(self.moveit_msgs_RobotTrajectory, true)
  msg:deserialize(msg_bytes)
  return msg
end

---Get the number of seconds
--@treturn number
function Plan:getPlanningTime()
  return f.getPlannigTime(self.o)
end

---Convert a ros message: moveit_msgs/RobotTrajectory
--@tparam[opt] ros.Message trajectory_msg
--@return Positions, Velocities and Labels (name of each joint)
function Plan:convertTrajectoyMsgToTable(trajectory_msg) -- expect a Utils object
  local positions = {}
  local velocities = {}
  local accelerations = {}
  local efforts = {}
  local points = trajectory_msg.joint_trajectory.points

  for i=1,#points do
    positions[i] = points[i].positions
    velocities[i] = points[i].velocities
    accelerations[i] = points[i].accelerations
    efforts[i] = points[i].effort
  end
  return positions,velocities,accelerations,efforts
end

---Convert a ros message: moveit_msgs/RobotState
--@tparam[opt] ros.Message start_state
--@return current position, velocity and labels (name of each joint)
function Plan:convertStartStateMsgToTensor(start_state) -- expect a Utils object
  local position = start_state.joint_state.position
  local velocity = start_state.joint_state.velocity
  local labels = start_state.joint_state.names
  return position, velocity, labels
end

local function plot6DTrajectory(trajectory)
    if trajectory[1]:nDimension()==0 then 
      return false 
    end
    local history_size = #trajectory   
 
    local q1={}
    local q2={}
    local q3={}
    local q4={}
    local q5={}
    local q6={}
    for i=1,history_size do
      q1[#q1+1] = trajectory[i][1]
      q2[#q2+1] = trajectory[i][2]
      q3[#q3+1] = trajectory[i][3]
      q4[#q4+1] = trajectory[i][4]
      q5[#q5+1] = trajectory[i][5]
      q6[#q6+1] = trajectory[i][6]
    end
    local q1_tensor = torch.Tensor(q1)
    local q2_tensor = torch.Tensor(q2)
    local q3_tensor = torch.Tensor(q3)
    local q4_tensor = torch.Tensor(q4)
    local q5_tensor = torch.Tensor(q5)
    local q6_tensor = torch.Tensor(q6)
    gnuplot.plot({'q1',q1_tensor}, {'q2',q2_tensor}, {'q3',q3_tensor},{'q4',q4_tensor},{'q5',q5_tensor},{'q6',q6_tensor})
  --gnuplot.axis{0, history_size, -100, 100}
  gnuplot.grid(true)
  return true
end

---Creates gnu plot for either position, velocity, acceleration and speed depending input
--@tparam int type if 1: Positions are plotted, 2: velocities are plotted,3: accelarations are plotted,4: speed is plotted
--@treturn boolean is true if the requested type of the plot is know.
function Plan:plot(type)
  local msg
  msg= self:getTrajectoryMsg()--Position
  local positions,velocities,accelerations,efforts= self:convertTrajectoyMsgToTable(msg)
  gnuplot.figure(type)

  if type == 1 then
    if plot6DTrajectory(positions)then 
      gnuplot.title('Trajectory position Data')
    else
      return false
    end
  elseif type == 2 then
    if plot6DTrajectory(velocities) then
      gnuplot.title('Trajectory velocity Data')
    else
      return false
    end
  elseif type == 3 then
    if plot6DTrajectory(accelerations) then
      gnuplot.title('Trajectory accelaration Data')
    else
      return false
    end
  elseif type ==4 then
    if velocities[1]:nDimension()==0 then
      return false 
    end
    history_size = #velocities
    local q = {}
    for i=1,#velocities do
      q[#q+1] = torch.norm(velocities[i])
    end
    local q_tensor = torch.Tensor(q)
    gnuplot.plot({'speed',q_tensor})
    --gnuplot.axis{0, history_size, -100, 100}
    gnuplot.grid(true)
    gnuplot.title('Trajectory speed Data')
  else
    print("plot type not yet implemented")
    return false
  end

  return true
end
