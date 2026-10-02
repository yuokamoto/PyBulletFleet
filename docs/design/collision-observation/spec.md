# Collision observation — scope draft

**Status:** Scope and architecture approved; implementation complete, awaiting review.

## Problem

The corridor evaluator needs to distinguish a positive-gap safety-margin entry
from geometric overlap. PBF exposes active object-ID pairs and a cumulative
entry count, but not the signed distance or detection method behind a pair.
The example therefore accesses PyBullet body IDs and calls `getClosestPoints`
for every robot/robot and robot/wall pair after each step. Other fleet scenarios
could need the same simulator-owned observation without repeating this internal
access or paying for a second all-pairs pass.

PR #52 corrected candidate extraction for margin-qualified pairs. It did not
add an observation API. The corridor experiment's policy, congestion and task
metrics remain outside PBF.

## Proposed goal

Expose a read-only snapshot of the most recent completed collision check's
qualifying pairs, with stable PBF object IDs, detection method, configured
margin, check step/time and signed minimum distance where the method supplies
one. State clearly whether a record comes from `CLOSEST_POINTS`,
`CONTACT_POINTS`, or the selected `HYBRID` branch. The snapshot must not
initiate another collision check or imply that an unsampled short event was
observed. Pair-specific distance queries are a separate option, not required
for this slice.

Prefer retaining the result already returned by the narrow phase over querying
all pairs again. Before implementation, check the cost and lifetime of those
results and the meaning of contact-point distance. If retaining them materially
changes core responsibilities or performance, return for Architecture Review.

## Draft acceptance criteria

1. An external application can read the most recent qualifying pair facts
   through a public PBF API without using PyBullet body IDs or private maps.
2. A closest-point record distinguishes positive gap from zero/negative
   geometric overlap and identifies the margin and check cadence. Contact-mode
   records do not claim an unsupported distance interpretation.
3. Records update when a pair enters, persists or exits and cannot masquerade
   as observations from a later simulation step when collision checks are less
   frequent than steps.
4. Tests cover closest-point, contact and hybrid modes, margin zero, a
   positive-gap pair, removal, and a skipped-check interval.
5. A representative fleet benchmark compares step/check cost and retained
   memory before and after the change. The corridor example can replace its
   private access without moving its episode or congestion logic into PBF.

## Non-goals

No generalized collision framework, collision verdict, automatic stop/reset,
all-pairs distance guarantee, policy framework, trace system, playback or
checkpoint. Keeping historical collision episodes and aggregating metrics
remain application responsibilities.

## Scope decision

Human approved recent qualifying-pair facts as the next slice. Pair-specific
on-demand distance queries remain a separate follow-up. See [plan](plan.md).
