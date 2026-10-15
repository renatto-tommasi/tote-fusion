# B2 evidence bundle

This directory retains compact evidence for the fresh numerical confirmation and
isolated native ROS replay. It does not contain hardware trials or qualify a
continuous client. [Results](../../results-confirmation-20261015.md),
[session](../../session-ros-20261015.md), [contract](../../ros-contract.md).

## Layout and identities

- confirmation: 30,000 original rows compressed without changing their contents,
  full summary and manifest, 1,200-scene cache index, generation/replay transcripts,
  and 2,400 worker logs in supporting-artifacts.tar.gz. Independent corner and
  primary-metric supplements retain missing-geometry denominators.
- ros-controls: Thirty positive/fault requests, original summary/manifest,
  compressed rows and supporting artifacts, including source, contract and context
  lifecycle records.
- ros-preflight: Sixty all-condition requests with the same evidence structure.
- ros-principal: The complete 540-request principal pilot, with the same evidence
  structure and 543 supporting files, including every native log and closed-context record.
- diagnostics.tar.gz: Failed preflights, CDR equality probe, fixed-count transport
  diagnostics, driver logs and diagnostic helper sources. diagnostics-index.json
  maps each of its 455 members to its original path and SHA256. The invalid
  duplicate-context-init arm and helper SyntaxError are preserved and excluded.
- review: The original blocked review and the later remedy review have separate
  reports, manifests and complete artifact archives. Their statuses are historical
  and remain unchanged. Final population verification is a separate report.

Each completed run has provenance.json with the uncompressed row hash and original
directory. Its original manifest hashes the uncompressed requests.jsonl. To check
that hash, decompress the published .gz file first. supporting-artifacts.tar.gz
contains the other files referenced by that manifest. The manifests are copied
byte for byte, including their original local paths.

The large pixels.npz cache files remain in the development workspace, outside Git.
The published index identifies every input/truth digest. The frozen generator,
seed ranges and environment are documented in the procedures. Independent review
audited the actual cached bytes, not only their index. No large native executable
or full simulator workspace is included.

Numerical source hashes match B1 implementation 5e890d2; its run began at 3fd75de.
The native adapter and control checker are c1702ad, with precursor 76ff1f3.
The production estimator source/configuration remains at 434803c. Reviewer evidence
retains source copies and exact executable identities. Revision links become
available after the associated commits are pushed.

## Integrity check

Run this command in this directory after checkout:

```bash
sha256sum -c SHA256SUMS
```

The outer index covers every published file except SHA256SUMS itself. Inner
manifests retain the original run/review identities. Raw logs retain original
whitespace. Do not normalize or edit those sealed files.

No incomplete run is merged into a completed population. A native rejection is a
result; a byte, scheduling, transport or setup failure is a protocol failure.
The [session](../../session-ros-20261015.md) describes the distinction and the
unresolved production admission-recovery path.
