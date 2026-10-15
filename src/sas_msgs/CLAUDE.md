# sas_msgs — rules

- `msg/SasPing.msg` is a frozen contract with the downstream processing package (ADR 0003).
- Do not add, remove, rename, or retype a field without a new ADR in `docs/decisions/` that
  supersedes 0003. Comment-only edits are allowed.
- Units go in a trailing comment on every numeric field.
- Echo layout stays element-major: `index = k * num_samples + n`.
