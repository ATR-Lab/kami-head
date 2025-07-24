cmake_minimum_required(VERSION 3.8)
project(sui_indexer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  sui_indexer/indexer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME indexer_node
)

install(PROGRAMS
  scripts/setup_prisma.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prisma_setup
)

# Install Prisma schema
install(DIRECTORY
  prisma/
  DESTINATION share/${PROJECT_NAME}/prisma
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package() 