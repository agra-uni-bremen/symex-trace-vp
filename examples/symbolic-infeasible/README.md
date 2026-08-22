# symbolic-infeasible

The fixture for **unsatisfiable solver queries** — solver time that produced no path.

Every other example here has essentially none. `Trace::findNewPath` negates one un-negated branch,
usually gets a satisfying assignment on the first try, and returns, so those traces contain about one
query per run and every one succeeds:

| Example | runs | queries | unsat |
|---|---|---|---|
| `symbolic-loop` | 16 | 15 | **0** |
| `assertion-failure` | 3 | 2 | **0** |
| `symbolic-sensor` | 2 | 2 | 1 |
| **`symbolic-infeasible`** | **7** | **13** | **7** |

That makes the other three useless for developing or testing anything that reports where solver time
went: "seconds spent proving no path exists" is structurally zero in them, so a consumer that
computed it wrongly — or swapped the `sat` flag — would look correct.

`main.c` pairs conditions that are each satisfiable alone and contradictory together (`a > 100` then
`a < 50`). Once the outer condition is in the path condition, negating the inner one asks for a value
that cannot exist. `findNewPath` marks that branch negated, pays for the UNSAT result, and moves on
to another branch, so the cost is real and no run comes of it. A final parity test is deliberately
feasible, so the trace is a **mix** rather than uniformly infeasible.

## What it demonstrates

Run with `SYMEX_SEED=42`, the per-gap structure comes out as:

```
 gap  queries   sat  unsat
   0        1     1      0
   1        1     1      0
   2        2     1      1
   3        1     1      0
   4        2     1      1
   5        4     1      3     <- three failed attempts before one worked
   6        2     0      2     <- terminal gap: proves the tree is exhausted, produces nothing
```

Three properties worth asserting against, all structural rather than timing-dependent:

- **Every gap except the last ends with exactly one satisfiable query, as its last.** That is what
  `findNewPath`'s `while (!assign.has_value())` guarantees, and it is the invariant any per-gap
  grouping depends on.
- **The final gap has no satisfiable query at all.** It is the exploration proving there is nothing
  left to negate, and it produces no run — so a consumer that assumes every gap yields a path will
  be wrong at exactly one place, at the end, where it is easy not to notice.
- **Run 0 has no query.** The first run executes concretely; there is nothing to solve for yet. So
  satisfiable queries number one fewer than runs.

As everywhere else here, **assert on counts, never on durations** — the `seconds` values are
wall-clock and swing by a factor of two or more between runs. With `SYMEX_SEED` set, the run and
query *sequence* is reproducible; the timings still are not.

## Building

```bash
PATH=/opt/riscv/bin:$PATH make
SYMEX_SEED=42 symex-vp main > symbolic-infeasible-query.rtrace
```

Build in a copy rather than here — `make` drops `.o` files and the `main` binary next to the sources.
