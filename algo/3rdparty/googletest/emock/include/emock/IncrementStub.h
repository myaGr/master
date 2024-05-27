/***
   emock is a cross-platform easy-to-use C++ Mock Framework based on mockcpp.
   Copyright [2017] [ez8.co] [orca <orca.zhang@yahoo.com>]

   This library is released under the Apache License, Version 2.0.
   Please see LICENSE file or visit https://github.com/ez8-co/emock for details.
   
   mockcpp is a C/C++ mock framework.
   Copyright [2008] [Darwin Yuan <darwin.yuan@gmail.com>]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
***/

#ifndef __EMOCK_INCREAMENT_STUB_H
#define __EMOCK_INCREAMENT_STUB_H

#include <emock/emock.h>
#include <emock/TypelessStub.h>
#include <emock/Formatter.h>
#include <emock/OutputStringStream.h>
#include <emock/Asserter.h>

EMOCK_NS_START

template <typename T>
struct IncrementStub : public TypelessStub
{
    IncrementStub(T startValue, T endValue)
       : from(startValue), to(endValue), current(startValue), hasEnd(true)
    {}

    IncrementStub(T startValue)
       : from(startValue), current(startValue), hasEnd(false)
    {}

    bool isCompleted() const
    {
       if(!hasEnd) 
       {
           return false;
       }

       return current > to; 
    }

    Any& invoke(void)
    {
       checkEnd();
       value = Any(current);
       current++;
       return value;
    }

    std::string toString(void) const
    {
       oss_t oss;

       oss << "increase(" << from;

       if(hasEnd)
       {
         oss << " ," << to;
       }

       oss << ")";

       return oss.str();
    }

    const std::type_info& type() const
    {
        return typeid(T);
    }

private:

   void checkEnd()
   {
      if(!hasEnd)
      {
         return;
      }
      oss_t oss;
      oss << "Returned value has increased to specified end value"
            << "(" << to << ")";
      EMOCK_ASSERT_TRUE_MESSAGE(
           oss.str(),
           current <= to);
   }

private:

    Any value;

    bool hasEnd;

    T current;
    T from;
    T to;
};

EMOCK_NS_END

#endif

