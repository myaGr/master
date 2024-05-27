/***
   emock is a cross-platform easy-to-use C++ Mock Framework based on mockcpp.
   Copyright [2017] [ez8.co] [orca <orca.zhang@yahoo.com>]

   This library is released under the Apache License, Version 2.0.
   Please see LICENSE file or visit https://github.com/ez8-co/emock for details.

    mockcpp is a generic C/C++ mock framework.
    Copyright (C) <2009>  <Darwin Yuan: darwin.yuan@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

#include <testcpp/testcpp.hpp>
#include <emock/IncrementStub.h>
#include <emock/types/AnyCast.h>

USING_EMOCK_NS

struct TestIncrementStub: public TESTCPP_NS::TestFixture
{
   void testShouldBeAbleIncreaseFromStartValue()
   {
      IncrementStub<int> stub(5);

      TS_ASSERT_EQUALS(5, any_cast<int>(stub.invoke()));
      TS_ASSERT_EQUALS(6, any_cast<int>(stub.invoke()));
      TS_ASSERT_EQUALS(7, any_cast<int>(stub.invoke()));
      TS_ASSERT_EQUALS(8, any_cast<int>(stub.invoke()));
   }

   void testShouldThrowExceptionAfterReachingItsEndValue()
   {
      IncrementStub<int> stub(5,7);

      TS_ASSERT_EQUALS(5, any_cast<int>(stub.invoke()));
      TS_ASSERT_EQUALS(6, any_cast<int>(stub.invoke()));
      TS_ASSERT_EQUALS(7, any_cast<int>(stub.invoke()));
      TS_ASSERT_THROWS(any_cast<int>(stub.invoke()), Exception);
      TS_ASSERT_THROWS(any_cast<int>(stub.invoke()), Exception);
   }
};
