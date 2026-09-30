# Fork-specific design and implementation notes

This directory contains documentation specific to the `hal1278/PocketSDR`
fork.

Files outside this directory under `doc/` are treated as upstream PocketSDR
documentation unless explicitly stated otherwise. Fork-specific design
decisions, implementation notes, experiments, and planned extensions should
be documented here instead of being mixed into upstream documents.

Documents:

- [doa_design.md](doa_design.md): initial DOA/multipath visualization design.
  This is the design baseline that preceded the current implementation.
- [doa_implementation.md](doa_implementation.md): current fork implementation
  status, deviations from the initial design, and remaining extension points.
- [array_calibration.md](array_calibration.md): array-calibration findings and
  the planned selectable calibration architecture.

The separation is intentional: upstream documentation should remain easy to
compare with upstream PocketSDR, while fork-specific work can evolve
independently.
