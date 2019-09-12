set(CMAKE_CXX_STANDARD 14)
# Remove Wno-unused function once all zmq  functions we need are used
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -lasan -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-ignored-qualifiers")
