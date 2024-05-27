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

#include <emock/types/Any.h>
#include <emock/Stub.h>
#include <emock/Asserter.h>
#include <emock/Invocation.h>
#include <emock/InvocationId.h>
#include <emock/DefaultMatcher.h>
#include <emock/StubsMatcher.h>
#include <emock/ExpectsMatcher.h>
#include <emock/InvocationMocker.h>
#include <emock/InvokedTimesMatcher.h>
#include <emock/InvocationMockerSet.h>
#include <emock/InvokedTimesRecorder.h>
#include <emock/InvocationTimesMatcher.h>
#include <emock/ChainableMockMethodCore.h>
#include <emock/SimpleInvocationRecorder.h>

EMOCK_NS_START

//////////////////////////////////////////////////////////
struct ChainableMockMethodCoreImpl
{
    std::string methodName;
    /////////////////////////////////////////////////////
    InvocationMockerSet mockers;
    InvocationMockerSet defaultMockers;
    InvocationMockerNamespace* invocationMockerNamespace;

	 /////////////////////////////////////////////////////
    ChainableMockMethodCoreImpl(const std::string& name, InvocationMockerNamespace* ns)
            : methodName(name), invocationMockerNamespace(ns) {}

    ~ChainableMockMethodCoreImpl();
   
    InvocationMocker* getInvocationMocker(const std::string& id) const;
    std::string tellNoMatchedExpectation(const Invocation& inv);
    std::string toString() const;
    const Any& invoke(const Invocation& inv, SelfDescribe* &resultProvider);
    void reset();
    void verify();
};

/////////////////////////////////////////////////////////////
std::string ChainableMockMethodCoreImpl::toString() const
{
    return mockers.toString() + defaultMockers.toString();
}

/////////////////////////////////////////////////////////////
void ChainableMockMethodCoreImpl::verify()
{
    mockers.verify();
}

/////////////////////////////////////////////////////////////
ChainableMockMethodCoreImpl::~ChainableMockMethodCoreImpl()
{
    reset();
}

/////////////////////////////////////////////////////////////
void ChainableMockMethodCoreImpl::reset()
{
    mockers.reset();
    defaultMockers.reset();
}
 
/////////////////////////////////////////////////////////////
InvocationMocker*
ChainableMockMethodCoreImpl::
getInvocationMocker(const std::string& id) const
{
    return mockers.getInvocationMocker(id);
}

/////////////////////////////////////////////////////////////
std::string
ChainableMockMethodCoreImpl::
tellNoMatchedExpectation(const Invocation& inv)
{
    return std::string("\n")
       + "=====================================\n"
       + std::string("Unexpected invocation: the invocation cannot be found in allowed invoking list.") + "\n" 
       + "Invoked: " + methodName + inv.toString() + "\n"
       + "Allowed: \n"
       + this->toString()
       + "=====================================\n";
}

/////////////////////////////////////////////////////////////
const Any&
ChainableMockMethodCoreImpl::invoke(const Invocation& inv
                                   , SelfDescribe* &resultProvider)
{
    const Any& result1 = mockers.invoke(inv, resultProvider);
    if (!result1.empty())
    {
      return result1;
    }

    const Any& result2 = defaultMockers.invoke(inv, resultProvider);
    if (!result2.empty())
    {
      return result2;
    }

	 EMOCK_FAIL(tellNoMatchedExpectation(inv));
   
	 return getEmptyAny();
}

//////////////////////////////////////////////////////////
ChainableMockMethodCore::
ChainableMockMethodCore(const std::string& name, InvocationMockerNamespace* ns)
   : This(new ChainableMockMethodCoreImpl(name, ns))
{
}

//////////////////////////////////////////////////////////
ChainableMockMethodCore::~ChainableMockMethodCore()
{
    delete This;
}

//////////////////////////////////////////////////////////
std::string&
ChainableMockMethodCore::getName() const
{
   return This->methodName;
}

//////////////////////////////////////////////////////////
const Any&
ChainableMockMethodCore::invoke
             ( const std::string& nameOfCaller
             , const RefAny& p1
             , const RefAny& p2
             , const RefAny& p3
             , const RefAny& p4
             , const RefAny& p5
             , const RefAny& p6
             , const RefAny& p7
             , const RefAny& p8
             , const RefAny& p9
             , const RefAny& p10
             , const RefAny& p11
             , const RefAny& p12
             , SelfDescribe* &resultProvider)
{
    Invocation inv(nameOfCaller,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12);
    return This->invoke(inv, resultProvider);
}

//////////////////////////////////////////////////////////
InvocationMocker*
ChainableMockMethodCore::getInvocationMocker(const std::string& id)
{
    return This->getInvocationMocker(id);
}

//////////////////////////////////////////////////////////
void
ChainableMockMethodCore::addInvocationMocker(InvocationMocker* mocker)
{
    This->mockers.addInvocationMocker(mocker);
}

//////////////////////////////////////////////////////////
void
ChainableMockMethodCore::addDefaultInvocationMocker(InvocationMocker* mocker)
{
    This->defaultMockers.addInvocationMocker(mocker);
}

//////////////////////////////////////////////////////////
InvocationMockerNamespace* ChainableMockMethodCore::getNamespace() const
{
    return This->invocationMockerNamespace;
}

//////////////////////////////////////////////////////////
void ChainableMockMethodCore::reset()
{
    This->reset();
}

//////////////////////////////////////////////////////////
void ChainableMockMethodCore::verify()
{
    This->verify();
}

//////////////////////////////////////////////////////////

EMOCK_NS_END

