cmake_minimum_required(VERSION 3.14)
project(autoware_claw)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()

# ── Python package ──
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  autoware_claw/autoware_mcp_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME autoware_mcp_server
)

# ── Config / Launch / Docker runtime ──
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(FILES docker-compose.yml DESTINATION share/${PROJECT_NAME}/docker)

# ── Docker container build (NemoClaw) ──
option(BUILD_DOCKER "Build/pull Docker containers for NemoClaw" ON)

if(BUILD_DOCKER)
  find_program(DOCKER_CMD docker)
  if(DOCKER_CMD)
    set(_docker_sources
      ${CMAKE_CURRENT_SOURCE_DIR}/docker-compose.yml
      ${CMAKE_CURRENT_SOURCE_DIR}/docker/nemoclaw.Dockerfile
      ${CMAKE_CURRENT_SOURCE_DIR}/docker/autoware-overlay/inject-mcp-config.py
      ${CMAKE_CURRENT_SOURCE_DIR}/docker/autoware-overlay/autoware-mcp-policy.yaml
    )
    set(_docker_stamp ${CMAKE_CURRENT_BINARY_DIR}/docker_build.stamp)

    # Build Docker image (NemoClaw only — Ollama removed, using OpenAI API)
    add_custom_command(
      OUTPUT ${_docker_stamp}
      COMMAND ${DOCKER_CMD} compose -p autoware_claw -f ${CMAKE_CURRENT_SOURCE_DIR}/docker-compose.yml build
      COMMAND ${CMAKE_COMMAND} -E touch ${_docker_stamp}
      DEPENDS ${_docker_sources}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Running docker compose build for autoware_claw (NemoClaw)..."
    )

    # # Ollama model pull removed — chat agent uses OpenAI API directly
    # set(_model_stamp ${CMAKE_CURRENT_BINARY_DIR}/ollama_model.stamp)
    # add_custom_command(
    #   OUTPUT ${_model_stamp}
    #   COMMAND ${DOCKER_CMD} compose -p autoware_claw -f ${CMAKE_CURRENT_SOURCE_DIR}/docker-compose.yml
    #           --profile setup run --rm -T ollama-pull
    #   COMMAND ${CMAKE_COMMAND} -E touch ${_model_stamp}
    #   DEPENDS ${_docker_stamp}
    #   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #   COMMENT "Pulling gemma4 model into ollama-data volume (skipped if already cached)..."
    # )

    add_custom_target(${PROJECT_NAME}_docker ALL DEPENDS ${_docker_stamp})
  else()
    message(WARNING "Docker not found — skipping NemoClaw container build. "
                    "Install Docker or set -DBUILD_DOCKER=OFF to silence this warning.")
  endif()
endif()

# ── Test ──
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
