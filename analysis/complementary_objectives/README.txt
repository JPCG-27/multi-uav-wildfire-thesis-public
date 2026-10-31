COMPLEMENTARY MISSION OUTCOMES

I report separate mission outcomes alongside the original cell-belief coverage,
without choosing a new weighted score to favour one planner. This reanalysis
uses the same 22,680 final ideal-sensor runs: seven policies, 3,240 runs each.
Camera-team studies remain separate because their sensing conditions differ.

REPRODUCE
From the repository root, using Python 3.10 or newer:
  python analysis/complementary_objectives/reproduce.py
Only the Python standard library and files in this directory are needed.
The command checks input hashes and regenerates per-run scores, method means,
paired effects, seed-block effects, allocation effects and results.json.
It does not launch Unreal or execute additional simulator runs.

EVIDENCE
run_evidence.csv.gz contains the 22,680 run records.
person_evidence.csv.gz contains 529,200 person-run records.
scenario_person_states.csv.gz contains 37,800 scenario people from 1,620
human trajectories. trajectory_provenance.csv.gz links these to source hashes.
provenance.json records input hashes and source identity. Raw method IDs remain
stable join keys; method_names.json supplies the reader-facing policy names.

DEFINITIONS
The horizon T is 8,000 ticks; every recorded appearance tick is zero.
Timeliness = 100 * mean(T + 1 - D) / T, where D is the first cell-belief
credit tick if it strictly precedes death, and 8,001 otherwise. This is the
normalised discrete area under cumulative first-credit-before-death coverage.
It is an affine rescaling of penalised before-death delay, not independent
evidence. A qualifying credit remains counted after a later death.

Recorded belief uptime = 100 * sum(T + 1 - first_credit - gap_ticks)/(N*T)
over credited people. Never-credited people contribute zero. First credit here
has no death restriction. The full horizon includes pre-credit absence.

Recorded untracked danger = accumulated untracked active-fire ticks / N.
The recorded danger is binary, so the weighted-exposure counter duplicates it.
Uptime and danger include retained post-death states. They are not alive-only
monitoring measures or proof of identity-confirmed visual contact.

First-fire time is the first positive sensor observation, with 8,001 for a
miss. The observed-fire fraction is also reported. This measures discovery,
not sustained fire-front tracking. Recorded belief uses positive accumulated
cell evidence and a recent observation; residual evidence can qualify credit.

COMPARISONS
Means give each run equal weight. Paired comparisons first average matched
differences within each of 15 seed blocks, then use a Student-t 95% interval.
Intervals are exploratory and unadjusted for multiple comparisons.
Outputs include every policy, grid size and allocation regime. Allocation
effects compare dynamic minus static; policy effects compare method minus
Priority planner. The source cohorts and build-scope qualifications remain
those of the final experiments; this analysis does not erase those differences.

FOV-aware lawnmower has the highest non-privileged mean person-credit timeliness.
Priority planner has the earliest non-privileged mean fire observation; its
contrast against Without FOV scoring has an interval that includes zero.
Without FOV scoring has slightly lower pooled untracked danger than Priority.
These distinct outcomes do not establish a universal best policy.
