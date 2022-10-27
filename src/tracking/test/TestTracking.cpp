/******************************************************************************
 *  @file       TestTracking.cpp
 *
 *  <!-- Start of section for manual description -->
 *  @brief      UnitTest for Tracking
 *  <!-- End of section for manual description -->
 *
 *  @author      Stefan Sass <stefan.sass@ovgu.de>
 *
 ******************************************************************************/
#include "gtest/gtest.h"

#include "tracking.h"


//////// ACTUAL TESTS ////////

// Test split string
TEST(TestSuite,test_case_1) {

    // serial input
    std::string inputString = "sub1,sub2 , sub3, ****.123434";
    std::vector<std::string> fields;
    fields = Tracking::splitString(inputString, ',');

    // expect 4 fields
    EXPECT_TRUE(fields.size()==4) << "[Test failed]: Expected = 4, Actual = "<< fields.size();
    // field 1
    EXPECT_TRUE(fields[0]=="sub1") << "[Test failed]: Expected = sub1, Actual = "<< fields[0];
    // field 2
    EXPECT_TRUE(fields[1]=="sub2 ") << "[Test failed]: Expected = sub2, Actual = "<< fields[1];
    // field 3
    EXPECT_TRUE(fields[2]==" sub3") << "[Test failed]: Expected = sub3, Actual = "<< fields[2];
    // field 4
    EXPECT_TRUE(fields[3]==" ****.123434") << "[Test failed]: Expected =  ****.123434, Actual = "<< fields[3];
}

// Test split string
TEST(TestSuite,test_case_2) {

    // serial input
    std::string inputString = "sub1,sub2 , sub3, ****.123434";
    std::vector<std::string> fields;
    fields = Tracking::splitString(inputString, ';');

    // expect 4 fields
    EXPECT_TRUE(fields.size()==1) << "[Test failed]: Expected = 1, Actual = "<< fields.size();
    // field 1 equal inputString
    EXPECT_TRUE(fields[0]==inputString) << "[Test failed]: Expected = " << inputString << ", Actual = "<< fields[0];

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}