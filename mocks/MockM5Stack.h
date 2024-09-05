// MockM5Stack.h
#pragma once

#ifndef M5STACKMOCK_H
#define M5STACKMOCK_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "M5Stack.h" // 実際のM5Stackクラスを継承する場合

class MockM5Stack : public M5Stack {
public:
    MOCK_METHOD(bool, begin, (), (override));
    MOCK_METHOD(void, setTextSize, (int size), (override));
    MOCK_METHOD(void, setCursor, (int x, int y), (override));
};

extern MockM5Stack M5;

#endif // M5STACKMOCK_H