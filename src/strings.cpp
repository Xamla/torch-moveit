#include "torch-moveit.h"
#include "utils.h"

MOVIMP(StringsPtr *, Strings, new)()
{
  return new StringsPtr(new std::vector<std::string>());
}

MOVIMP(StringsPtr *, Strings, clone)(StringsPtr *self)
{
  const std::vector<std::string>& v = **self;
  return new StringsPtr(new std::vector<std::string>(v));
}

MOVIMP(void, Strings, delete)(StringsPtr *ptr)
{
  delete ptr;
}

MOVIMP(int, Strings, size)(StringsPtr *self)
{
  const std::vector<std::string>& v = **self;
  return static_cast<int>(v.size());
}

MOVIMP(const char*, Strings, getAt)(StringsPtr *self, size_t pos)
{
  std::vector<std::string>& v = **self;
  return v[pos].c_str();
}

MOVIMP(void, Strings, setAt)(StringsPtr *self, size_t pos, const char *value)
{
  std::vector<std::string>& v = **self;
  v[pos] = value;
}

MOVIMP(void, Strings, push_back)(StringsPtr *self, const char *value)
{
  std::vector<std::string>& v = **self;
  v.push_back(value);
}

MOVIMP(void, Strings, pop_back)(StringsPtr *self)
{
  std::vector<std::string>& v = **self;
  v.pop_back();
}

MOVIMP(void, Strings, clear)(StringsPtr *self)
{
  std::vector<std::string>& v = **self;
  v.clear();
}

MOVIMP(void, Strings, insert)(StringsPtr *self, size_t pos, size_t n, const char *value)
{
  std::vector<std::string>& v = **self;
  std::vector<std::string>::iterator i;
  
  if (pos >= v.size())
    i = v.end();
    
  v.insert(i, n, value);
}

MOVIMP(void, Strings, erase)(StringsPtr *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;
    
  std::vector<std::string>& v = **self;
  std::vector<std::string>::iterator b, e;
  if (begin >= v.size())
    b = v.end();
  else
    b = v.begin() + begin;
  if (end >= v.size())
    e = v.end();
  else
    e = v.begin() + end;
  v.erase(b, e);
}

MOVIMP(bool, Strings, empty)(StringsPtr *self)
{
  const std::vector<std::string>& v = **self;
  return v.empty();
}
