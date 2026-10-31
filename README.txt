MULTI-UAV WILDFIRE SEARCH AND RESCUE — DISTRIBUTION EDITION

Supporting materials for my University of Minho master's dissertation on
multi-UAV coordination in wildfire search and rescue.
Repository: https://github.com/JPCG-27/multi-uav-wildfire-thesis-public
This distribution edition is held private until submission. Its name describes
the intended distribution; it does not imply that access is currently public.
The complete private research archive is maintained separately.

CONTENTS
analysis/                 Run-level records and portable numerical reanalysis.
simulator/main_evaluated/  Retained source, map and configuration for the main study.
simulator/current/         Current source, maps and runtime/evaluation tools.
detector/                 Recorded metrics, predictions and training environment.
paths/                    Index of 42 selected ideal-sensor flight paths.
dependencies/             Historical asset inventory and acquisition notes.
Release attachments       Scenario CSVs, configurations, selected traces,
                          supplemental provenance and recorded camera responses.

WHAT IS EXCLUDED
Third-party art bundles, detector images/labels/masks and split metadata, and
both the initial and final detector checkpoints are excluded. No private asset
or detector dataset ZIP is listed in the download manifests. This repository
starts with fresh history; the withheld files are not in earlier commits.
See THIRD_PARTY_NOTICES.txt and dependencies/README.txt. Historical records may
still identify an excluded asset, dataset path or checkpoint hash: those are
provenance, not supplied files or permission to reuse the original inputs.

QUICK NUMERICAL REANALYSIS
No Unreal installation, detector dataset, checkpoint or release download is
needed for these seven analysis entrypoints. Use Python 3.12:
  python -m pip install -r requirements-analysis.txt
  python analysis/analyze_main_results.py
  python analysis/analyze_camera_cohorts.py
  python analysis/verify_camera_derived_summaries.py
  python analysis/verify_live_render_hashes.py
  python analysis/verify_live_stage2_hashes.py
  python analysis/data/ideal_methods/reproduce_numbers.py
  python analysis/complementary_objectives/reproduce.py

The seven-method table covers 22,680 runs, 3,240 per method, with 1,620 scenarios
under static and dynamic allocation (50-tick updates). Equal-run mean cell-belief
coverage, rounded:
  Priority planner       92.402%
  Without FOV scoring    87.904%
  Random target          80.133%
  Fully random           50.741%
  Lawnmower              74.554%
  FOV-aware lawnmower    95.837%
  Oracle                 99.958%
method_names.json maps recorded identifiers to dissertation labels. Confidence
intervals use paired seed-block means. Camera team studies are separate from
the 22,680 ideal-sensor runs. Cell-belief coverage is not individual visual
discovery. Complementary-outcome definitions retain the distinction between
before-death credit, recorded belief uptime and fire discovery; see the README
in analysis/complementary_objectives.

REPRODUCTION ATTACHMENTS
Release tag: reproduction-package. The three attachment manifests list exactly
13 ZIPs: eight scenario-input parts, one run-configuration archive, one selected
flight-path archive, one supplemental provenance archive and two camera-response
archives. Install GitHub CLI; authentication is required while the repo is private.
  python fetch_release.py --group scenario-inputs --extract
  python fetch_release.py --group run-configurations --extract
  python fetch_release.py --group flight-paths --extract
  python fetch_release.py --group supplemental-run-configurations-and-provenance --extract
  python fetch_release.py --group camera-response-cache
Run without --group to download and verify all 13 ZIPs. Each independent ZIP
is checked against its byte size and SHA-256 before extraction. --archive-dir
can point to an existing local directory for offline verification/extraction;
it never downloads missing files in that mode. --repo and --tag can select a
mirror retaining exactly the same manifest checksums.

SIMULATOR SOURCES AND EXECUTION REQUIREMENTS
This is not a self-contained runnable Unreal scene. Install Unreal Engine 5.7,
its Windows C++ prerequisites and the documented plugins (ModelingToolsEditorMode,
SunPosition, GeoReferencing, NiagaraFluids). The retained project/template maps
and Blueprints can refer to excluded art. Obtain the required assets separately
under appropriate licences before attempting to load and run the scenes.
Using substitutes changes the visual experiment and is not exact reproduction.

Build the selected Thesis.uproject. Use /Game/NewMap explicitly, a recorded
configuration, and absolute executable/project/configuration paths:
  UnrealEditor-Cmd.exe <Thesis.uproject> /Game/NewMap -game -RenderOffscreen -unattended -nosplash -nosound -SimConfig=<configuration.json>
Do not use -nullrhi for live camera rendering. Scenario archives restore
Saved/Benchmarks/_scenarios under simulator/current. For main_evaluated, copy
that exact scenario tree and separately acquired required Content to its root.
Relocate path-valued fields in a copy of the configuration, preserve the
scientific settings and use a new output directory. A clean-machine rebuild
and fresh simulator execution have not been validated for this reduced package.

SOURCE IDENTITY
Use the source associated with each experiment. The main study's retained source
is in main_evaluated; current contains the later project. The source-publication
manifest records formatting/comment changes with original and distributed hashes.
Supplemental source/build records describe their cohort scope. They do not supply
an independently measured executable hash per run. The dependency inventory is
historical evidence, not a verified asset snapshot for every evaluated experiment.

RECORDED CAMERA RESPONSES
After downloading the camera-response-cache groups:
  python restore_camera_cache.py --verify-only
Omit --verify-only to restore millions of response JSONs. Use --cohort
mixed_camera or four_camera and --limit-runs for a smaller check. The original
experiments used record mode. Replay requires a copy of the matching config,
vision_cache_mode set to replay and vision_cache_dir relocated to the restored
project. Preserve the remaining settings. These numeric response records are
neither camera images nor a substitute training dataset. They do not restore
the excluded scene assets or trained detector needed for fresh inference.

DETECTOR RECORDS
The recorded final split contained 1,153 training, 194 validation and 210 test
images; those images and their labels are not distributed here. The recorded
environment includes CPython 3.12.10, Ultralytics 8.4.75, PyTorch 2.12.1+cu130
and RTX 5070 Ti. The validation-selected confidence threshold was
0.25306564569473267. Test metrics and archived predictions are retained as
scientific records. Image-level rescoring requires the withheld labels/inputs;
training, continuation and fresh inference are not reproducible from this
distribution alone. Obtain appropriate rights before using any external
dataset/model with the supplied detector tools. Do not retune on the test split.

SCOPE
The seven analysis entrypoints reproduce retained numerical results and repeat
comparisons. Image hashes compare sampled recorded images; complete camera
streams are not included. Temporal-shift tables omit some original per-frame
transforms and rank-level intermediates. Full ownership streams and four-camera
production flight paths are not supplied. The 42 traces are selected ideal-sensor
paths. Cross-machine/driver equality is not established by these records.

LICENSING
No blanket licence is assigned to this mixed-material repository. Original
research material, Unreal template content and external tools retain their own
terms. Public availability does not grant unrestricted reuse. The withheld
datasets/checkpoints and art must not be uploaded to this repository or release.
See THIRD_PARTY_NOTICES.txt for component boundaries and upstream terms.
