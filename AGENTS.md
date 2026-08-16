# AGENTS.md

<!-- fm-render:begin agents-invariants sha256:2341b3e06285c5ba588cd3126694fa705e0a8c07e7cd91fd7c3f61c45ba0bc68 — rendered by the First Motive render plane — edit the upstream source, not this file -->
## First Motive Invariants

These four hold in every First Motive repo. Break one and the review, the CI
check, or the next machine catches it — usually all three.

- **Names.** Repos, packages, and hosts are `fm-<kebab>`; Python modules are
  `fm_<snake>`. A name that does not carry the `fm` prefix is unreachable to the
  `fm` CLI and to the tooling that discovers repos by prefix.
- **Config, never source.** Anything that differs per host — hostname, role,
  workspace path, transport, device IDs — is read from `machine.json`, never
  typed into a script, unit file, or launch file. A hardcoded host value works on
  exactly one machine and silently breaks the rest of the fleet.
- **Commits.** Subject line only: `prefix: phrase`, lowercase imperative, no
  body, no trailers. Prefixes: `init`, `feat`, `fix`, `docs`, `refactor`,
  `chore`. A commit body is dropped by the repo's hook, so anything explained
  there is lost.
- **Python through uv.** `uv run`, `uv add`, `uv sync` — never bare `python`,
  `pip`, `poetry`, or `virtualenv`. A bare invocation resolves against whatever
  interpreter the shell happens to have, which is why "works on my machine"
  reports are almost always this.
<!-- fm-render:end agents-invariants -->
