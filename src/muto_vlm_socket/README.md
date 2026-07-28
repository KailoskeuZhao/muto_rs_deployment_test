# Muto VLM socket

`muto_vlm_socket` is a ROS 2 action bridge to an OpenAI-compatible multimodal
endpoint. It supports both the Responses API and Chat Completions. It has no
camera subscription and no fixed prompt: each caller supplies one ordered
request containing any interleaving of text and JPEG parts.

## Interface

The `/vlm/generate` action accepts:

```text
GenerateVlm goal
  content[]
    type = TYPE_TEXT  -> text is populated, jpeg_data is empty
    type = TYPE_JPEG  -> jpeg_data is populated, text is empty
  model               -> empty uses default_model
  response_json_schema -> empty requests text; otherwise a JSON Schema object
```

Order is preserved. These are all valid requests:

- one text part;
- one JPEG part;
- JPEG followed by a question;
- instructions, multiple JPEGs, then another text part.

The entire ordered array becomes one OpenAI-compatible `user` message. The
result contains response text and prompt/completion token counts when supplied
by the endpoint. Progress feedback covers validation, connection, model wait,
and response decoding.

When `response_json_schema` is nonempty, the socket requests strict structured
output. It maps the schema to Responses `text.format` or Chat Completions
`response_format`, depending on `wire_api`. The schema must describe a root
object and is bounded by `max_text_characters` and `max_request_bytes`.
Incomplete, failed, refused, or content-filtered generations fail the action;
partial text is never returned as a successful result.

Only one request is executed at a time; overlapping goals are rejected. Action
cancellation closes the active HTTP connection on a best-effort basis, while
`request_timeout` provides the hard upper bound.

## Credentials and launch

No API key parameter exists, so credentials cannot accidentally appear in YAML,
ROS parameter introspection, or launch arguments. Export the configured
environment variable before launch:

```bash
export DASHSCOPE_API_KEY='your-key'
ros2 launch muto_vlm_socket vlm_socket_launch.py \
  base_url:=http://vlm-host:8000/v1 \
  wire_api:=responses \
  default_model:=gpt-5.5
```

`wire_api` accepts `responses` (the default) or `chat_completions`. The socket
appends `/responses` or `/chat/completions` to `base_url` when the suffix is not
already present. Responses requests use `store: false` by default; change
`store_response` only in a parameter file when retention is intentional.

For an unauthenticated local endpoint, set `require_api_key: false` in a custom
parameter file.

Text-only goals can be sent from the CLI:

```bash
ros2 action send_goal /vlm/generate \
  muto_vlm_socket/action/GenerateVlm \
  "{content: [{type: 1, text: 'Describe your capabilities.', jpeg_data: []}], model: '', response_json_schema: ''}" \
  --feedback
```

For JPEG requests, use a client node rather than expressing binary data on the
command line:

```python
from pathlib import Path

from muto_vlm_socket.action import GenerateVlm
from muto_vlm_socket.msg import VlmContent

goal = GenerateVlm.Goal()

image = VlmContent()
image.type = VlmContent.TYPE_JPEG
image.jpeg_data = Path('/tmp/view.jpg').read_bytes()

question = VlmContent()
question.type = VlmContent.TYPE_TEXT
question.text = 'What obstacles are visible?'

goal.content = [image, question]
goal.model = ''
goal.response_json_schema = ''
# Send goal with rclpy.action.ActionClient.
```

## Safety and resource limits

The server validates the tagged-union fields, JPEG start/end markers, content
count, combined text size, per-image size, total image size, encoded request
size, response size, URL scheme, and timeout before returning data to callers.
It never logs prompts, JPEG bytes, response text, authorization headers, or
server response bodies.

JPEG bytes travel through DDS as part of the action goal. The default 8 MiB
per-image limit is an application ceiling, not a guarantee that every DDS
configuration accepts a message that large. Normal compressed camera frames are
preferable; adjust both these parameters and middleware limits when necessary.
