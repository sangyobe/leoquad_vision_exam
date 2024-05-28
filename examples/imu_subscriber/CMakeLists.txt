project(imu_subscriber)

set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

file(GLOB SRCS "*.cpp" "${CMAKE_SOURCE_DIR}/proto/*.cc")
message(STATUS "${PROJECT_NAME} sources: ${SRCS}")
add_executable(${PROJECT_NAME}
    ${SRCS}
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
    dtproto_grpc
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    spdlog::spdlog
    yaml-cpp
)