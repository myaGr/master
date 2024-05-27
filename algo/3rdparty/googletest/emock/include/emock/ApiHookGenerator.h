/***
   emock is a cross-platform easy-to-use C++ Mock Framework based on mockcpp.
   Copyright [2017] [ez8.co] [orca <orca.zhang@yahoo.com>]

   This library is released under the Apache License, Version 2.0.
   Please see LICENSE file or visit https://github.com/ez8-co/emock for details.
   
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

#ifndef __EMOCK_API_HOOK_GENERATOR_H__
#define __EMOCK_API_HOOK_GENERATOR_H__

#include <emock/ApiHookFunctor.h>

EMOCK_NS_START

/////////////////////////////////////////////////////////////////
template <typename F, unsigned int Seq>
struct ApiHookGenerator
{
    static void* findApiHook(F* api)
    {
        void* hook;

        (hook = ApiHookFunctor<F, Seq>::getApiHook(api)) ||
        (hook = ApiHookGenerator<F, Seq-1>::findApiHook(api));

        return hook;
    }

    static void* applyApiHook(F* api)
    {
       void* hook;

       (hook = ApiHookFunctor<F, Seq>::applyApiHook(api)) ||
       (hook = ApiHookGenerator<F, Seq-1>::applyApiHook(api));

       return hook;
    }

    static bool freeApiHook(void* hook)
    {
        return 
        (ApiHookFunctor<F, Seq>::freeApiHook(hook)) ||
        (ApiHookGenerator<F, Seq-1>::freeApiHook(hook));
    }

private:

};

/////////////////////////////////////////////////////////////////
template <typename F>
struct ApiHookGenerator<F, 0>
{
    static void* findApiHook(F* api)
    { return 0; }

    static void* applyApiHook(F* api)
    { 
        oss_t oss;
        
        oss << "Did you define too many mockers in a testcase? "
            << "Probably you should refine your design, "
            << "or you can reconfig ParameterizedApiHookHolder::maxSeq bigger, "
            << "it's the number of mockers in a testcase that with the same function prototype, "
            << "the bigger it is, the slower compiling is.";

        EMOCK_REPORT_FAILURE(oss.str());
   
        return 0; 
    }

    static bool freeApiHook(void* hook)
    { return true; }
};

/////////////////////////////////////////////////////////////////

EMOCK_NS_END

#endif

