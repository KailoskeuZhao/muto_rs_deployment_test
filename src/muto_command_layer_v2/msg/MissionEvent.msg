# Append-only semantic event committed by the executive.
builtin_interfaces/Time stamp
uint64 sequence
string schema_version
string mission_id
string request_id
uint64 board_revision

string event_type
string lifecycle_state
string skill
string tool
string outcome
string reason_code
string candidate_id
string registry_revision
string detail
string evidence_id
string evidence_source
float32 evidence_confidence
float64 evidence_timestamp_s
# JSON arrays of request terms; confirmation requires complete matched terms
# and an empty unmatched list.
string evidence_matched_attributes_json
string evidence_unmatched_attributes_json
