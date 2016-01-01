local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

std = moveit.std

local String = torch.class('std.String', std)

local f

function init()
  local String_method_names = {
    "new",
    "clone",
    "delete",
    "assign",
    "length",
    "c_str"
  }
  
  f = utils.create_method_table("std_string_", String_method_names)
end

init()

function String:__init(s)
  if type(s) == 'string' then
    self.o = f.new(s, #s)
  else
    self.o = f.new(ffi.NULL, 0)
  end
end

function String:cdata()
  return self.o
end

function String:assign(s)
  s = tostring(s)
  f.assign(self.o, s, #s)
end

function String:length()
  return f.length(self.o)
end

function String:get()
  return ffi.string(f.c_str(self.o), f.length(self.o))
end

function String:__len()
  return self:length()
end

function String:__tostring()
  return self:get()
end
