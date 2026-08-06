# Copyright 2026 kailoskeuzhao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(MUTO_RS_GIT_REVISION "unknown")
set(MUTO_RS_GIT_DIRTY "false")

if(GIT_EXECUTABLE AND EXISTS "${GIT_EXECUTABLE}")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${REPOSITORY_DIR}" rev-parse HEAD
    OUTPUT_VARIABLE MUTO_RS_GIT_REVISION_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE MUTO_RS_GIT_REVISION_RESULT
    ERROR_QUIET
  )
  if(MUTO_RS_GIT_REVISION_RESULT EQUAL 0 AND MUTO_RS_GIT_REVISION_OUTPUT)
    set(MUTO_RS_GIT_REVISION "${MUTO_RS_GIT_REVISION_OUTPUT}")
  endif()

  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${REPOSITORY_DIR}" status --porcelain
    OUTPUT_VARIABLE MUTO_RS_GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE MUTO_RS_GIT_STATUS_RESULT
    ERROR_QUIET
  )
  if(MUTO_RS_GIT_STATUS_RESULT EQUAL 0 AND MUTO_RS_GIT_STATUS)
    set(MUTO_RS_GIT_DIRTY "true")
  endif()
endif()

get_filename_component(OUTPUT_DIRECTORY "${OUTPUT_PATH}" DIRECTORY)
file(MAKE_DIRECTORY "${OUTPUT_DIRECTORY}")
configure_file("${TEMPLATE_PATH}" "${OUTPUT_PATH}" @ONLY)
