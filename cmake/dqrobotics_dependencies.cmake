if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics_dependencies)
set (CMAKE_CXX_STANDARD 11)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE)
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    # The library is installed here when using the regular cmake ., make, sudo make install
    LINK_DIRECTORIES(
        /usr/local/lib/)
endif()

if(WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES) 
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED) 
    INCLUDE_DIRECTORIES(
	${EIGEN3_INCLUDE_DIR}
	# Default path when using cmake .., cmake --build ., cmake --install .
	"C:/Program Files (x86)/dqrobotics/include"
	) 
	LINK_DIRECTORIES(
	# Default path when using cmake .., cmake --build ., cmake --install .
    "C:/Program Files (x86)/dqrobotics/lib"
	)
endif()