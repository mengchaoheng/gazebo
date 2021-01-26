find_package(Protobuf REQUIRED)

set (msgs ductedfan.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(ductedfan_msgs SHARED ${PROTO_SRCS})
target_link_libraries(ductedfan_msgs ${PROTOBUF_LIBRARY})
