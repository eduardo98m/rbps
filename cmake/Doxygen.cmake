# =============================================================================
#  Doxygen.cmake — opt-in API documentation target
#
#  Usage (from top-level CMakeLists.txt):
#    option(RBPS_BUILD_DOCS "Build API documentation with Doxygen" OFF)
#    if(RBPS_BUILD_DOCS)
#        include(cmake/Doxygen.cmake)
#    endif()
#
#  Then:  cmake --build build --target docs
#  Output: build/docs/html/index.html
# =============================================================================

find_package(Doxygen REQUIRED)

include(FetchContent)

# ── Doxygen Awesome CSS — modern theme (single CSS file drop-in) ─────────────
FetchContent_Declare(doxygen_awesome
    GIT_REPOSITORY https://github.com/jothepro/doxygen-awesome-css.git
    GIT_TAG        v2.3.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(doxygen_awesome)

set(DOXYGEN_AWESOME_DIR "${doxygen_awesome_SOURCE_DIR}")
set(DOXYGEN_AWESOME_CSS "${DOXYGEN_AWESOME_DIR}/doxygen-awesome.css")
# Optional companion files (sidebar-only layout, dark-mode toggle assets).
set(DOXYGEN_AWESOME_EXTRA_FILES "")

# ── Inputs to Doxygen ────────────────────────────────────────────────────────
# Headers + the docs/ folder (so STYLE.md, README.md, design/*.md are picked up).
set(DOXYGEN_INPUT_DIRS
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/docs"
)
string(REPLACE ";" " " DOXYGEN_INPUT_DIRS "${DOXYGEN_INPUT_DIRS}")

set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs")
set(DOXYGEN_MAINPAGE   "${CMAKE_SOURCE_DIR}/docs/README.md")

# Project version — bump as the project version changes.
if(NOT DEFINED RBPS_VERSION)
    set(RBPS_VERSION "0.1.0")
endif()

# ── Generate Doxyfile from template ──────────────────────────────────────────
set(DOXYFILE_IN  "${CMAKE_SOURCE_DIR}/Doxyfile.in")
set(DOXYFILE_OUT "${CMAKE_BINARY_DIR}/Doxyfile")
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# ── docs target ──────────────────────────────────────────────────────────────
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

add_custom_target(docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
)

message(STATUS "[docs] target enabled — run: cmake --build ${CMAKE_BINARY_DIR} --target docs")
message(STATUS "[docs] output: ${DOXYGEN_OUTPUT_DIR}/html/index.html")
