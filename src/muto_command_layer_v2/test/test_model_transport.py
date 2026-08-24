import json

from muto_command_layer_v2.contracts import (
    CompletionPolicy,
    MissionBoard,
    SCHEMA_VERSION,
)
from muto_command_layer_v2.model_transport import (
    build_decision_schema,
    build_planner_prompt,
)


def test_prompt_is_compact_board_json_and_explicitly_marks_visual_input():
    board = MissionBoard(
        mission_id="mission-1",
        objective="find the chair",
        object_request="chair",
        completion_policy=CompletionPolicy.REPORT_CONFIRMED,
        registry_revision="r7",
        shortlisted_candidate_ids=("chair_1", "chair_2"),
    )
    prompt = build_planner_prompt(board, visual_available=True)
    assert "VISUAL_INPUT_AVAILABLE=true" in prompt
    payload = json.loads(prompt.split("STATE_JSON=", 1)[1])
    assert payload["mission_id"] == "mission-1"
    assert payload["shortlisted_candidate_ids"] == ["chair_1", "chair_2"]


def test_decision_schema_is_strict_and_matches_parser_enums():
    schema = json.loads(build_decision_schema())
    assert schema["additionalProperties"] is False
    assert schema["properties"]["schema_version"]["const"] == SCHEMA_VERSION
    assert schema["properties"]["skill"]["enum"] == [
        "search_for_object", "approach_confirmed_object"
    ]
    tool = schema["properties"]["tool"]["anyOf"][1]
    assert tool["properties"]["frame_id"]["const"] == "map"
