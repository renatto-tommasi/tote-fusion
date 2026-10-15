# Review evidence copy

`report.md`, `review.json`, provenance and the sealed reviewer manifest are exact
copies of the final independent review. `raw-review-evidence.tar.gz` contains all
other original review files, including tests, auditor scripts, mutations, failed
attempts, own replay rows and audit outputs. Paths in the report refer inside it.
The only excluded file is the source archive, reproducible from published Git:

```bash
git archive 5e890d2ac7159020f61075caa71f4873656a0e8c > source-5e890d2.tar
```

The full original bundle is retained locally at
`verification/paired_core_review_20261015` in the parent workspace.
