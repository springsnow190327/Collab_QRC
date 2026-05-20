# Response to a negotiation request.

std_msgs/Header header
string request_id
string requester_id
string responder_id
bool accepted
string reason
time response_stamp
ClaimedFrontier[] accepted_requester_claims
ClaimedFrontier[] accepted_responder_claims
time responder_last_interaction_attempt_stamp
uint8 protocol_version