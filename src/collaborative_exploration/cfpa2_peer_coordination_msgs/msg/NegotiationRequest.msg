# Sent when one robot proposes a new frontier allocation.

std_msgs/Header header
string request_id
string requester_id
string responder_id
builtin_interfaces/Time request_stamp
ClaimedFrontier[] requester_claims
ClaimedFrontier[] responder_claims
uint8 protocol_version
