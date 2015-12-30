local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local tf = moveit.tf

local Quaternion = torch.class('tf.Quaternion', tf)

local f

function init()
  local Quaternion_method_names = {
    "new",
    "clone",
    "delete",
    "setIdentity",
    "setRotation_Tensor",
    "setEuler",
    "setRPY",
    "getAngle",
    "getAxis_Tensor",
    "inverse",
    "length2",
    "normalize",
    "angle",
    "angleShortestPath",
    "add",
    "sub",
    "mul",
    "mul_scalar",
    "div_scalar",
    "dot",
    "slerp",
    "viewTensor"
  }
  
  f = utils.create_method_table("tf_Quaternion_", Quaternion_method_names )
end

init()

function Quaternion:__init(x,y,z,w)
  self.o = f.new()
  if x then
    if y and z and w then
      self:fromTensor(torch.DoubleTensor({x,y,z,w}))
    elseif torch.isTensor(x) then
      self:fromTensor(x)
    elseif torch.isTypeOf(x, tf.Quaternion) then
      self:fromTensor(x:toTensor())
    end
  end
end

function Quaternion:cdata()
  return self.o
end

function Quaternion:clone()
  local c = torch.factory('tf.Quaternion')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Quaternion:setIdentity()
  f.setIdentity(self.o)
end

function Quaternion:setRotation(axis, angle)
  if type(axis) == 'table' then
    axis = torch.DoubleTensor(axis)
  end
  if torch.isTensor(axis) then
    f.setRotation_Tensor(self.o, axis:cdata(), angle)
  else
    error('Tensor axis expected')
  end
end

function Quaternion:setEuler(yaw, pitch, roll)
  f.setEuler(self.o, yaw, pitch, roll)
end

function Quaternion:setRPY(roll, pitch, yaw)
  f.setRPY(roll, pitch, yaw)
end

function Quaternion:getAngle(angle)
  return f.getAngle(self.o)
end

function Quaternion:getAxis(axis)
  axis = axis or torch.DoubleTensor()
  f.getAxis_Tensor(self.o, axis:cdata())
  return axis
end

function Quaternion:inverse()
  result = tf.Quaternion()
  f.inverse(self.o, result:cdata())
  return result
end

function Quaternion:length2()
  return f.length2(self.o)
end

function Quaternion:length()
  return math.sqrt(self:length2())
end

function Quaternion:normalize()
  f.normalize(self.o)
end

function Quaternion:normalized()
  local c = self:clone()
  c:normalize()
  return c
end

function Quaternion:angle(other)
  return f.angle(self.o, other:cdata())
end

function Quaternion:angleShortestPath()
  return f.angleShortestPath(self.o, other:cdata())
end

function Quaternion:add(other, result)
  result = result or tf.Quaternion()
  f.add(self.o, other:cdata(), result:cdata())
  return result
end

function Quaternion:sub(other, result)
  result = result or tf.Quaternion()
  f.sub(self.o, other:cdata(), result:cdata())
  return result
end

function Quaternion:mul(other, result)
  result = result or tf.Quaternion()
  if torch.isTypeOf(other, tf.Quaternion) then
    f.mul(self.o, other:cdata(), result:cdata())
  elseif type(other) == 'number' then
    f.mul_scalar(self.o, other:cdata(), result:cdata())
  else
    error('Unsupported type of factor for quaternion multiplication.')
  end
  return result
end

function Quaternion:div(divisor, result)
  result = result or tf.Quaternion()
  if type(divisor) == 'number' then
    f.div_scalar(self.o, divisor, result:cdata())
  else
    error('Unsupported type of factor for quaternion division.')
  end
  return result
end

function Quaternion:dot(other)
  return f.dot(self.o, other:cdata())
end
    
function Quaternion:slerp(other, t, result)
  result = result or tf.Quaternion()
  f.slerp(self.o, other:cdata(), t, result:cdata())
  return result
end

function Quaternion:fromTensor(t)
  local v = self:toTensor()
  v[{}] = t
end

function Quaternion:toTensor()
  local result = torch.DoubleTensor()
  f.viewTensor(self.o, result:cdata())
  return result
end

function Quaternion:__tostring()
  local t = self:toTensor()
  return string.format("{ x:%f, y:%f, z:%f, w:%f }", t[1], t[2], t[3], t[4]);
end
