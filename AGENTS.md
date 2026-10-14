# Contribution instructions

Read `TOTE_FUSION_NODE_SPEC.md` and `docs/design_decisions.md` before changing the estimator.

Value robustness, minimum total lines of code, no bloat and no duplication.
Do not add bug-hiding fallbacks or useless conditionals. Broken internal
invariants must fail loudly; prefer assertions. Validate external input with
release-active checks. Expected sensor faults must return explicit rejection,
never guessed data or apparent success.

Keep simulator truth outside estimator inputs and dependencies. Publish
candidates only, using acquisition-time TF, immutable model revisions and
bounded request/cancel/failure behavior. Retain meaningful test, mutation and
simulation evidence and document remaining limits. The present profiles are
simulation-only; hardware claims require separate authorization and validation.
