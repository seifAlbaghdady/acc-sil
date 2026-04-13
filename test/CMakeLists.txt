find_package(GTest REQUIRED)

add_executable(test_acc test_acc.cpp)
target_link_libraries(test_acc PRIVATE acc_static GTest::GTest GTest::Main)

include(GoogleTest)
gtest_discover_tests(test_acc)
