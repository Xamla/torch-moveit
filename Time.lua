local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'
local ros = moveit.ros

local Time = torch.class('ros.Time', ros)

local f

function init()
  local Time_method_names = {
    "new",
    "clone",
    "delete",
    "isZero",
    "fromSec",
    "toSec",
    "set",
    "get_sec",
    "set_sec",
    "get_nsec",
    "set_nesc",
    "lt",
    "eq",
    "add_Duration",
    "sub",
    "sub_Duration",
    "sleepUntil",
    "getNow",
    "setNow",
    "waitForValid",
    "init",
    "shutdown",
    "useSystemTime",
    "isSimTime",
    "isSystemTime",
    "isValid"
  }
  
  f = utils.create_method_table("ros_Time_", Time_method_names )
end

init()

function Time:__init()
  self.o = f.new()
end

function Time:cdata()
  return self.o
end

function Time:clone()
  local c = torch.factory('ros.Transform')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Time:isZero()
  return f.isZero(self.o)
end

function Time:fromSec(t)
  f.fromSec(self.o, t)
end

function Time:toSec()
  return f.toSec(self.o)
end

function Time:set(sec, nsec)
  f.set(self.o, sec, nsec)
end

function Time:get_sec()
  return f.get_sec(self.o)
end

function Time:set_sec(sec)
  f.set_sec(self.o, sec)
end
  
function Time:get_nsec()
  return f.get_nsec(self.o)
end

function Time:set_nesc(nsec)
  f.set_nesc(self.o, nsec)
end

function Time:__lt(other)
  return f.lt(self.o, other:cdata())
end

function Time:__eq(other)
  return f.eq(self.o, other:cdata())
end

function Time:add(d, result)
  result = result or self
  f.add_Duration(self.o, d:cdata(), result:cdata())
  return result
end

function Time:sub(x, result)
  if torch.isTypeOf(x, ros.Time) then
    result = result or ros.Duration()
    f.sub(self.o, x:cdata(), result:cdata())
  elseif torch.isTypeOf(x, ros.Duration) then
    result = result or self
    f.sub_Duration(self.o, x:cdata(), result:cdata())
  else
    error('cannot sub from ros.Time with specified argument type')
  end
  return result
end

function Time:__add(d)
  local result = ros.Time()
  return self:add(d, result)
end

function Time:__sub(x)
  local result
  if torch.isTypeOf(x, ros.Time) then
    result = ros.Duration()
    f.sub(self.o, x:cdata(), result:cdata())
  elseif torch.isTypeOf(x, ros.Duration) then
    result = ros.Time()
    f.sub_Duration(self.o, x:Duration(), result:cdata())
  end
  return result
end

function Time:__tostring()
  return string.format("{ sec: %d, nsec: %d }", self:get_sec(), self:get_nsec())
end

-- static functions

function Time.init()
  f.init()
end

function Time.shutdown()
  f.shutdown()
end

function Time.getNow()
  local now = ros.Time()
  f.getNow(now:cdata())
  return now
end

function Time.setNow(time)
  f.setNow(time:cdata())
end

function Time.sleepUntil(time)
  f.sleepUntil(time:cdata())
end

function Time.waitForValid()
  f.waitForValid()
end

function Time.useSystemTime()
  return f.useSystemTime()
end

function Time.isSimTime()
  return f.isSimTime()
end

function Time.isSystemTime()
  return f.isSimTime()
end

function Time.isValid()
  return f.isValid()
end
