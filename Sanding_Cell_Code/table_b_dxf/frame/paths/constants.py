from __future__ import annotations

# Keep frame tool TCP away from the raw model boundary unless a connector only touches a corner.
FRAME_BOUNDARY_CLEARANCE_MM = 3.0

# Offset-centreline rails can create a tiny backtracking corner stub. Up to this length is
# treated as a negligible corner artifact, not intentional re-sanding.
OFFSET_CORNER_STUB_MM = 60.0

# A connector may touch the outer edge briefly, but must not run along it.
MAX_OUTER_EDGE_CONTACT_MM = 15.0
